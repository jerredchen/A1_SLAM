"""
Compute the biases of the IMU when the A1 is stationary.
"""

from collections import deque
from typing import List, Deque

import matplotlib.pyplot as plt
import numpy as np
import rosbag

import gtsam
from gtsam.symbol_shorthand import B, V, X

fig, axes = plt.subplots(2, 3)
fig.set_tight_layout(True)

def report_progress(result: gtsam.Values, key_count: int, plotted_biases: Deque):
    """Print the current estimate of a stationary A1 as well as plot the estimated bias of the IMU."""

    # Print the estimate of the A1.
    print("*"*50 + f"\nInference after State {key_count}:\n")
    print("Pose:\n")
    print(result.atPose3(X(key_count)), '\n')
    print("Velocity:\n")
    print(result.atVector(V(key_count)), '\n')
    print("Bias:\n")
    print(result.atConstantBias(B(0)), '\n')

    accel_labels = ['x', 'y', 'z']
    accel_colors = ['r', 'b', 'g']
    gyro_labels = ['roll', 'pitch', 'yaw']
    gyro_colors = ['m', 'c', 'y']

    for i, label in enumerate(accel_labels):
        ax = axes[0][i]
        ax.clear()
        ax.set_xlabel(f'acceleration bias in {label}')
        for b in plotted_biases:
            key = b[0]
            estimate = b[1].accelerometer()[i]
            ax.scatter(key, estimate, color=accel_colors[i])

    for i, label in enumerate(gyro_labels):
        ax = axes[1][i]
        ax.clear()
        ax.set_xlabel(f'gyroscope bias in {label}')
        for b in plotted_biases:
            key = b[0]
            estimate = b[1].gyroscope()[i]
            ax.scatter(key, estimate, color=gyro_colors[i])

    plt.pause(0.01)

def report_final_biases(all_biases: List):
    """Plot the final bias estimates across the entire alignment."""

    fig, axes = plt.subplots(2, 3)
    fig.set_tight_layout(True)

    accel_labels = ['x', 'y', 'z']
    accel_colors = ['r', 'b', 'g']
    gyro_labels = ['roll', 'pitch', 'yaw']
    gyro_colors = ['m', 'c', 'y']

    keys = [b[0] for b in all_biases]
    for i, label in enumerate(accel_labels):
        ax = axes[0][i]
        ax.clear()
        ax.set_title(f'Acceleration Bias in {label}')
        ax.set_xlabel('Key Number')
        ax.set_ylabel('Bias (m/s^2)')
        estimates = [b[1].accelerometer()[i] for b in all_biases]
        ax.plot(keys, estimates, color=accel_colors[i])

    for i, label in enumerate(gyro_labels):
        ax = axes[1][i]
        ax.clear()
        ax.set_title(f'Gyroscope Bias in {label}')
        ax.set_xlabel('Key Number')
        ax.set_ylabel('Bias (deg/s)')
        estimates = [b[1].gyroscope()[i] for b in all_biases]
        ax.plot(keys, estimates, color=gyro_colors[i])
    
    plt.show()

    print(all_biases[-1][1])

def optimize_trajectory(bag_name: str,
                        topic_name: str,
                        initial_bias = gtsam.imuBias.ConstantBias(),
                        prior_pose_noise = gtsam.noiseModel.Diagonal.Sigmas(
                            np.array([1e-5, 1e-5, 1e-5, 1e-6, 1e-6, 1e-6])),
                        prior_vel_noise = gtsam.noiseModel.Isotropic.Sigma(3, 1e-5),
                        prior_bias_noise = gtsam.noiseModel.Isotropic.Sigma(6, 1e-3)):

    # Open the ROS bag to be used.
    bag = rosbag.Bag(bag_name)

    # Instantiate the factor graph and its initial estimates container.
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()

    # Instantiate the iSAM2 parameters to create the iSAM2 object.
    parameters = gtsam.ISAM2Params()
    parameters.setRelinearizeThreshold(0.1)
    parameters.setRelinearizeSkip(1)
    isam = gtsam.ISAM2(parameters)

    # Instantiate the PIM parameters to create the PIM object.
    pim_params = gtsam.PreintegrationParams.MakeSharedD(-9.81)
    pim_params.setAccelerometerCovariance(np.eye(3) * 1e-4)
    pim_params.setGyroscopeCovariance(np.eye(3) * 1e-4)
    pim_params.setIntegrationCovariance(np.eye(3) * 1e-4)
    pim = gtsam.PreintegratedImuMeasurements(pim_params, initial_bias)

    # Add the prior factors to the factor graph, and initialize the estimates.
    graph.add(gtsam.PriorFactorPose3(X(0), gtsam.Pose3(), prior_pose_noise))
    initial_estimate.insert(X(0), gtsam.Pose3())
    graph.add(gtsam.PriorFactorVector(V(0), np.array([0, 0, 0]), prior_vel_noise))
    initial_estimate.insert(V(0), np.array([0, 0, 0]))
    graph.add(gtsam.PriorFactorConstantBias(B(0), initial_bias, prior_bias_noise))
    initial_estimate.insert(B(0), initial_bias)

    key_count = 1
    plotted_biases = deque([], maxlen=10)
    all_biases = []

    for k, bag_info in enumerate(bag.read_messages(topic_name)):

        # Extract the IMU measurements from the bag at a particular time instance.
        msg = bag_info[1]
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

        # Preintegrate the acceleration and angular velocity to obtain an unoptimized estimate of the state.
        measured_accel = np.array([accel_x, accel_y, accel_z])
        measured_omega = np.array([omega_x, omega_y, omega_z])
        pim.integrateMeasurement(measured_accel, measured_omega, t - t_prev)
        t_prev = t

        if k % 10 == 0:

            # Add IMU factor to the factor graph.
            imu_factor = gtsam.ImuFactor(X(key_count - 1), V(key_count - 1), X(key_count), V(key_count), B(0), pim)
            graph.push_back(imu_factor)

            # Add a prior to X to constrain the position to be Pose3().
            pos_factor = gtsam.PriorFactorPose3(X(key_count), gtsam.Pose3(), PRIOR_POSE_NOISE)
            graph.push_back(pos_factor)

            # Add a prior to V to constrain the velocity to be (0, 0, 0).
            vel_factor = gtsam.PriorFactorVector(V(key_count), np.array([0, 0, 0]), PRIOR_VEL_NOISE)
            graph.push_back(vel_factor)

            initial_estimate.insert(X(key_count), gtsam.Pose3())
            initial_estimate.insert(V(key_count), np.array([0, 0, 0]))
            pim.resetIntegration()

            # Perform an incremental update to estimate the IMU biases.
            isam.update(graph, initial_estimate)
            result = isam.calculateEstimate()
            plotted_biases.append((key_count, result.atConstantBias(B(0))))
            all_biases.append((key_count, result.atConstantBias(B(0))))
            if len(plotted_biases) > 10:
                plotted_biases.popleft()
            report_progress(result, key_count, plotted_biases)

            # Reset the factor graph and initial estimates container.
            key_count += 1
            graph = gtsam.NonlinearFactorGraph()
            initial_estimate.clear()
    bag.close()

    report_final_biases(all_biases)

if __name__ == "__main__":

    # Declare the pose's translational standard deviation, in meters.
    prior_xyz_pose_sigma = 1e-7

    # Declare the pose's rotational standard deviation, in degrees.
    prior_rpy_sigma = 1e-7

    # Declare the velocity's standard deviation in all directions, in m/s.
    prior_vel_sigma = 1e-7

    # Declare the bias' standard deviation in all directions, in m/s^2 or degrees/s.
    prior_bias_sigma = 0.25

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

    # Assume the initial biases of the IMU to be 0.
    initial_bias = gtsam.imuBias.ConstantBias()

    optimize_trajectory('data/bags/imu_alignment-2020-09-04.bag',
                        'IMU_data',
                        initial_bias=initial_bias,
                        prior_pose_noise=PRIOR_POSE_NOISE,
                        prior_vel_noise=PRIOR_VEL_NOISE,
                        prior_bias_noise=PRIOR_BIAS_NOISE)
