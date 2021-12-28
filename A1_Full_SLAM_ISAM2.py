"""
Incrementally parsing ROSbag to obtain Imu data and produce Imu factor graph solution.
"""
import numpy as np
import rosbag

import A1_LiDAR_ISAM2
import A1_Plot
import gtsam
from gtsam.symbol_shorthand import B, V, X
import Personal_ICP


def optimize_trajectory(bag_name: str,
                        imu_topic: str,
                        lidar_topic: str,
                        initial_bias: gtsam.imuBias.ConstantBias(),
                        prior_pose_noise = gtsam.noiseModel.Diagonal.Sigmas(
                            np.array([1e-5, 1e-5, 1e-5, 1e-6, 1e-6, 1e-6])),
                        prior_vel_noise = gtsam.noiseModel.Isotropic.Sigma(3, 1e-5),
                        prior_bias_noise = gtsam.noiseModel.Isotropic.Sigma(6, 1e-3),
                        icp_noise = gtsam.noiseModel.I):
    """Incrementally optimize the A1's trajectory using both IMU and LiDAR.

    Args:
        bag_name: The name of the ROS bag that was collected over the course of a trajectory.
        imu_topic: The name of the IMU topic which recorded the IMU measurements.
        lidar_topic: The name of the LiDAR topic which recorded the LiDAR measurements.
        initial_bias: The initial biases to be used as a prior for the IMU biases.
        prior_pose_noise: The noise model associated with the prior pose.
        prior_vel_noise: The noise model associated with the prior velocity.
        prior_bias_noise: The noise model associated with the initial biases.
    """

    # Open the ROS bag to be used.
    bag = rosbag.Bag(bag_name)

    # Instantiate the factor graph and its initial estimates container.
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()

    # Instantiate the iSAM2 parameters to create the iSAM2 object.
    isam_parameters = gtsam.ISAM2Params()
    isam_parameters.setRelinearizeThreshold(0.1)
    isam_parameters.setRelinearizeSkip(1)
    isam = gtsam.ISAM2(isam_parameters)

    # Instantiate the PIM parameters to create the PIM object.
    pim_parameters = gtsam.PreintegrationParams.MakeSharedD(0.0)
    pim_parameters.setAccelerometerCovariance(np.eye(3) * 1e-3)
    pim_parameters.setGyroscopeCovariance(np.eye(3) * 1e-3)
    pim_parameters.setIntegrationCovariance(np.eye(3) * 1e-3)
    pim = gtsam.PreintegratedImuMeasurements(pim_parameters)

    # Add the prior factors to the factor graph, and initialize the estimates.
    graph.add(gtsam.PriorFactorPose3(X(0), gtsam.Pose3(), prior_pose_noise))
    initial_estimate.insert(X(0), gtsam.Pose3())
    graph.add(gtsam.PriorFactorVector(V(0), np.array([1.0, 0.0, 0.0]), prior_vel_noise))
    initial_estimate.insert(V(0), np.array([1.0, 0.0, 0.0]))
    graph.add(gtsam.PriorFactorConstantBias(B(0), initial_bias, prior_bias_noise))
    initial_estimate.insert(B(0), initial_bias)
    isam.update(graph, initial_estimate)
    initial_estimate.clear()
    result = isam.calculateEstimate()

    scan_transform = gtsam.Pose3()
    skip_transform = gtsam.Pose3()
    key_count = 0
    state_prev = gtsam.NavState(gtsam.Pose3(), np.array([0, 0, 0]))
    bias_prev = initial_bias

    for k, bag_info in enumerate(bag.read_messages([imu_topic, lidar_topic])):

        topic = bag_info[0]
        msg = bag_info[1]
        # Extract the IMU measurement and perform preintegration if a LiDAR measurement
        # has not arrived yet.
        if topic == imu_topic:
            if k == 0:
                t_prev = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
                continue
            t = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
            accel_x = msg.linear_acceleration.x
            accel_y = msg.linear_acceleration.y
            accel_z = msg.linear_acceleration.z
            omega_x = msg.angular_velocity.x
            omega_y = msg.angular_velocity.y
            omega_z = msg.angular_velocity.z

            measured_accel = np.array([accel_x, accel_y, accel_z])
            measured_omega = np.array([omega_x, omega_y, omega_z])
            pim.integrateMeasurement(measured_accel, measured_omega, t - t_prev)
            t_prev = t

        # Add both an IMU factor and an odometry factor if the LiDAR measurement has arrived.
        elif topic == lidar_topic:
            if key_count == 1:
                scan_prev = scan
            elif key_count > 1:
                scan_pprev = scan_prev
                scan_prev = scan

            # Convert the scan from a 1D array of ranges to an array of "3D" points in the local pose frame.
            scan = A1_LiDAR_ISAM2.ranges_to_points(msg.ranges)

            if key_count > 0:
                # Add the IMU factor to the graph.
                imu_factor = gtsam.ImuFactor(X(key_count - 1), V(key_count - 1), X(key_count), V(key_count), B(0), pim)
                graph.add(imu_factor)

                predicted_state = pim.predict(state_prev, bias_prev)
                initial_estimate.insert(V(key_count), predicted_state.velocity())
                pim.resetIntegration()

                # Add the odometry factor between consecutive poses.
                scan_transform = Personal_ICP.icp(scan, scan_prev, initial_transform=scan_transform)
                graph.add(gtsam.BetweenFactorPose3(X(key_count - 1), X(key_count), scan_transform, icp_noise))
                initialized_odom = result.atPose3(X(key_count - 1)).compose(scan_transform)
                initial_estimate.insert(X(key_count), initialized_odom)
                if key_count > 1:
                    # Add the odometry factor between skipped (non-consecutive) poses for additional constraints.
                    skip_transform = Personal_ICP.icp(scan, scan_pprev, initial_transform=skip_transform)
                    graph.add(gtsam.BetweenFactorPose3(X(key_count - 2), X(key_count), skip_transform, icp_noise))
                    scan_pprev = scan_prev

                # Perform the incremental iSAM2 update.
                isam.update(graph, initial_estimate)
                result = isam.calculateEstimate()
                A1_Plot.plot_full_incremental_traj_and_map(result, scan, start=key_count, time_interval=0.01)

                # Reinitialize for the next iteration.
                state_prev = gtsam.NavState(result.atPose3(X(key_count)), result.atVector(V(key_count)))
                bias_prev = result.atConstantBias(B(0))
                scan_prev = scan
                graph = gtsam.NonlinearFactorGraph()
                initial_estimate.clear()

            key_count += 1

    bag.close()

if __name__ == "__main__":
    # Declare the pose's standard deviation in the xyz-direction, in meters.
    prior_xyz_pose_sigma = 1e-5

    # Declare the pose's roll-pitch-yaw rotational standard deviation, in degrees.
    prior_rpy_sigma = 1e-4

    # Declare the velocity's standard deviation in all directions, in m/s.
    prior_vel_sigma = 1e-5

    # Declare the bias' standard deviation in all directions, in m/s^2 or degrees/s.
    prior_bias_sigma = 1e-6

    # Declare the ICP noise standard deviation in the xyz-direction, in meters.
    icp_xyz_sigma = 1e-3

    # Declare the roll-pitch-yaw standard deviation for the ICP noise model, in degrees.
    icp_rpy_sigma = 1e-4

    # Instantiate noise models for the pose, velocity, and bias priors as well as for ICP, respectively.
    PRIOR_POSE_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([prior_rpy_sigma * np.pi / 180,
                                                                  prior_rpy_sigma * np.pi / 180,
                                                                  prior_rpy_sigma * np.pi / 180,
                                                                  prior_xyz_pose_sigma,
                                                                  prior_xyz_pose_sigma,
                                                                  prior_xyz_pose_sigma]))
    PRIOR_VEL_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, prior_vel_sigma)
    PRIOR_BIAS_NOISE = gtsam.noiseModel.Isotropic.Sigma(6, prior_bias_sigma)
    ICP_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([icp_rpy_sigma * np.pi / 180,
                                                           icp_rpy_sigma * np.pi / 180,
                                                           icp_rpy_sigma * np.pi / 180,
                                                           icp_xyz_sigma,
                                                           icp_xyz_sigma,
                                                           icp_xyz_sigma]))

    optimize_trajectory('data/bags/A1_bag_square_traj_2020-09-04-18-03-08.bag',
                        '/imu',
                        '/slamware_ros_sdk_server_node/scan',
                        prior_pose_noise=PRIOR_POSE_NOISE,
                        prior_vel_noise=PRIOR_VEL_NOISE,
                        prior_bias_noise=PRIOR_BIAS_NOISE,
                        icp_noise=ICP_NOISE)