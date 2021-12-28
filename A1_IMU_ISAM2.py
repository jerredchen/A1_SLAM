"""
Incrementally parsing ROSbag to obtain IMU data and produce IMU factor graph solution.
"""

import numpy as np
import rosbag

import A1_Plot
import gtsam
from gtsam.symbol_shorthand import B, V, X

def optimize_trajectory(bag_name: str,
                        topic_name: str,
                        imu_factor_rate = 50,
                        initial_bias = gtsam.imuBias.ConstantBias(),
                        is2D = False,
                        prior_pose_noise = gtsam.noiseModel.Diagonal.Sigmas(
                            np.array([1e-5, 1e-5, 1e-5, 1e-6, 1e-6, 1e-6])),
                        prior_vel_noise = gtsam.noiseModel.Isotropic.Sigma(3, 1e-5),
                        prior_bias_noise = gtsam.noiseModel.Isotropic.Sigma(6, 1e-3)):
    """Incrementally optimize the A1's trajectory from only using IMU measurements.

    Args:
        bag_name: The name of the ROS bag that was collected over the course of a trajectory.
        topic_name: The name of the IMU topic which consists of the recorded IMU measurements.
        imu_factor_rate: The number of IMU measurements preintegrated before adding a new IMU factor
            to the factor graph.
        is2D: Boolean parameter denoting whether the resulting optimized trajectory is 2D or 3D.
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
    pim_parameters = gtsam.PreintegrationParams.MakeSharedD(0)
    pim_parameters.setAccelerometerCovariance(np.eye(3) - 1e-5)
    pim_parameters.setGyroscopeCovariance(np.eye(3) - 1e-5)
    pim_parameters.setIntegrationCovariance(np.eye(3) - 1e-5)
    pim = gtsam.PreintegratedImuMeasurements(pim_parameters, initial_bias)

    # Add the prior factors to the factor graph, and initialize the estimates.
    graph.add(gtsam.PriorFactorPose3(X(0), gtsam.Pose3(gtsam.Pose2(4, 1, np.pi/2)), prior_pose_noise))
    initial_estimate.insert(X(0), gtsam.Pose3())
    graph.add(gtsam.PriorFactorVector(V(0), np.array([0.0, 0.0, 0.0]), prior_vel_noise))
    initial_estimate.insert(V(0), np.array([0.0, 0.0, 0.0]))
    graph.add(gtsam.PriorFactorConstantBias(B(0), initial_bias, prior_bias_noise))
    initial_estimate.insert(B(0), initial_bias)

    key_count = 1
    state_prev = gtsam.NavState(gtsam.Pose3(), np.array([0, 0, 0]))
    bias_prev = initial_bias

    for k, bag_info in enumerate(bag.read_messages(topic_name)):

        # Extract the IMU measurements from the bag at a particular time instance.
        msg = bag_info[1]
        if k == 0:
            t_prev = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
            continue
        t = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        # If only a 2D trajectory is considered, arbitrarily set the unused measurements to 0.
        if is2D:
            accel_z = 0
            omega_x = 0
            omega_y = 0
        else:
            accel_z = msg.linear_acceleration.z
            omega_x = msg.angular_velocity.x
            omega_y = msg.angular_velocity.y
        omega_z = msg.angular_velocity.z

        # Preintegrate the acceleration and angular velocity to obtain an unoptimized estimate of the state.
        measured_accel = np.array([accel_x, accel_y, accel_z])
        measured_omega = np.array([omega_x, omega_y, omega_z])
        pim.integrateMeasurement(measured_accel, measured_omega, t - t_prev)
        t_prev = t

        # Add an IMU factor to the factor graph after a specified number of IMU measurements have been preintegrated.
        if k % imu_factor_rate == 0:

            # Add the IMU factor to the factor graph.
            imu_factor = gtsam.ImuFactor(X(key_count - 1), V(key_count - 1), X(key_count), V(key_count), B(0), pim)
            graph.push_back(imu_factor)

            # Initialize the estimates for the pose and velocity by integrating from the previous state.
            predicted_state = pim.predict(state_prev, bias_prev)
            initial_estimate.insert(X(key_count), predicted_state.pose())
            initial_estimate.insert(V(key_count), predicted_state.velocity())
            pim.resetIntegration()

            # Perform the incremental update using iSAM2.
            isam.update(graph, initial_estimate)
            result = isam.calculateEstimate()

            A1_Plot.plot_incremental_IMU_progress(result, start=key_count, time_interval=0.05)

            # Reinitialize the previous states and biases as well as clearing the factor graph/initial estimates.
            state_prev = gtsam.NavState(result.atPose3(X(key_count)), result.atVector(V(key_count)))
            bias_prev = result.atConstantBias(B(0))
            key_count += 1
            graph = gtsam.NonlinearFactorGraph()
            initial_estimate.clear()

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

    # Instantiate noise models based on prior standard deviations for the pose,
    # velocity, and initial biases respectively.
    PRIOR_POSE_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([
            prior_rpy_sigma * np.pi / 180,
            prior_rpy_sigma * np.pi / 180,
            prior_rpy_sigma * np.pi / 180,
            prior_xyz_pose_sigma,
            prior_xyz_pose_sigma,
            prior_xyz_pose_sigma
        ]))
    PRIOR_VEL_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, prior_vel_sigma)
    PRIOR_BIAS_NOISE = gtsam.noiseModel.Isotropic.Sigma(6, prior_bias_sigma)

    optimize_trajectory('data/bags/turtlebot_imu_2021-12-17-13-45-03.bag',
                        '/imu',
                        is2D=False,
                        imu_factor_rate=50,
                        prior_pose_noise=PRIOR_POSE_NOISE,
                        prior_vel_noise=PRIOR_VEL_NOISE,
                        prior_bias_noise=PRIOR_BIAS_NOISE)
