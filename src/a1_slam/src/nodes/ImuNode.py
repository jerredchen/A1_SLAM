"""
Helper functions for setting up inertial odometry in the PoseSLAM factor graph.
"""
import rospy

import gtsam
import numpy as np
from unitree_legged_msgs.msg import HighState
from gtsam.symbol_shorthand import B, V, X


def _parse_config_parameters(
        prior_pose_estimate,
        prior_vel_estimate,
        prior_bias_estimate,
        prior_pose_sigmas,
        prior_vel_sigmas,
        prior_bias_sigmas,
        accel_sigmas,
        gyro_sigmas,
        integration_sigmas):
    """Parse the config parameters to be used as gtsam objects.

    Args:
        prior_pose_estimate:    A list in format [x, y, z, roll, pitch, yaw]
                                    representing the initial estimate on the prior pose.
        prior_vel_estimate:     A list in format [x, y, z]
                                    representing the initial estimate on the prior velocity.
        prior_bias_estimate:    A list in format [x, y, z, roll, pitch, yaw]
                                    representing the initial estimate on the prior biases.
        prior_pose_sigmas:      A list in format [x, y, z, roll, pitch, yaw] representing the prior
                                    pose noise's translational and rotational standard deviations.
        prior_vel_sigmas:       A list in format [x, y, z] representing the prior velocity noise's
                                    translational standard deviations.
        prior_bias_sigmas:      A list in format [x, y, z, roll, pitch, yaw] representing the prior
                                    bias noise's standard deviations.
        accel_sigmas:           A list in format [x, y, z]
                                    representing the accelerometer noise's standard deviations.
        gyro_sigmas:            A list in format [roll, pitch, yaw]
                                    representing the gyroscope noise's standard deviations.
        integration_sigmas:     A list in format [x, y, z]
                                    representing the integration noise's standard deviations.
    Returns:
        prior_pose_factor:      The prior factor associated with the initial pose.
        prior_vel_factor:       The prior factor associated with the initial velocity.
        prior_bias_factor:      The prior factor associated with the initial biases.
        accel_noise_cov:        The covariance associated with the accelerometer.
        gyro_noise_cov:         The covariance associated with the gyroscope.
        integration_noise_cov:  The covariance associated with the IMU integration.
    """
    rpy = np.deg2rad(prior_pose_estimate[3:])
    pose_rotation = gtsam.Rot3.Ypr(*(rpy[::-1]))
    pose_translation = gtsam.Point3(*prior_pose_estimate[:3])
    prior_pose_estimate = gtsam.Pose3(pose_rotation, pose_translation)

    prior_vel_estimate = np.array(prior_vel_estimate)

    accel_bias = prior_bias_estimate[:3]
    gyro_bias = prior_bias_estimate[3:]
    prior_bias_estimate = gtsam.imuBias.ConstantBias(accel_bias, gyro_bias)

    prior_pose_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.concatenate(
            (np.deg2rad(prior_pose_sigmas[3:]), prior_pose_sigmas[:3]))
    )
    prior_vel_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array(prior_vel_sigmas))
    prior_bias_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array(prior_bias_sigmas))

    prior_pose_factor = gtsam.PriorFactorPose3(
        X(0), prior_pose_estimate, prior_pose_noise)
    prior_vel_factor = gtsam.PriorFactorVector(
        V(0), prior_vel_estimate, prior_vel_noise)
    prior_bias_factor = gtsam.PriorFactorConstantBias(
        B(0), prior_bias_estimate, prior_bias_noise)

    accel_noise_cov = np.diag(np.square(accel_sigmas))
    gyro_noise_cov = np.diag(np.square(gyro_sigmas))
    integration_noise_cov = np.diag(np.square(integration_sigmas))

    factor_graph_params = (
        prior_pose_factor,
        prior_vel_factor,
        prior_bias_factor,
        accel_noise_cov,
        gyro_noise_cov,
        integration_noise_cov
    )
    return factor_graph_params


def _create_factor_graph_and_params(
        prior_pose_factor: gtsam.PriorFactorPose3,
        prior_vel_factor: gtsam.PriorFactorVector,
        prior_bias_factor: gtsam.PriorFactorConstantBias,
        accel_noise: gtsam.noiseModel,
        gyro_noise: gtsam.noiseModel,
        integration_noise: gtsam.noiseModel):
    """Create a factor graph and the necessary parameters for optimization.

    Args:
        prior_pose_factor:  The prior factor associated with the initial pose.
        prior_vel_factor:   The prior factor associated with the initial velocity.
        prior_bias_factor:  The prior factor associated with the initial biases.
        accel_noise:        A numpy array representing the covariance of the accelerometer.
        gyro_noise:         A numpy array representing the covariance of the gyroscope.
        integration_noise:  A numpy array representing the covariance of the IMU integration.
    Returns:
        graph:              A gtsam.NonlinearFactorGraph with the priors inserted.
        initial_estimate:   A gtsam.Values with the initial estimates stored.
        isam:               A iSAM2 object which will perform incremental inference.
        pim:                A PreintegrationImuMeasurements object performing the IMU preintegration.
    """

    # Instantiate the factor graph and its initial estimates container.
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()

    # Instantiate the PIM parameters to create the PIM object.
    pim_parameters = gtsam.PreintegrationParams.MakeSharedU()
    pim_parameters.setAccelerometerCovariance(accel_noise)
    pim_parameters.setGyroscopeCovariance(gyro_noise)
    pim_parameters.setIntegrationCovariance(integration_noise)
    pim = gtsam.PreintegratedImuMeasurements(
        pim_parameters, prior_bias_factor.prior())

    # Add the prior factors to the factor graph, and initialize the estimates.
    graph.add(prior_pose_factor)
    initial_estimate.insert(X(0), prior_pose_factor.prior())
    graph.add(prior_vel_factor)
    initial_estimate.insert(V(0), prior_vel_factor.prior())
    graph.add(prior_bias_factor)
    initial_estimate.insert(B(0), prior_bias_factor.prior())

    return graph, initial_estimate, pim


def create_imu_graph_and_params(graph, initial_estimates):
    """Parses the rosparams to setup the relevant IMU graph and parameters.
    Args:
        graph: The initialized factor graph.
        initial_estimates: The initialized initial estimates.
    """

    # Obtain the pose's translational and rotational standard deviations, in meters and degrees.
    prior_pose_sigmas = rospy.get_param('prior_pose_sigmas')

    # Obtain the velocity's standard deviation in all directions, in m/s.
    prior_vel_sigmas = rospy.get_param('prior_vel_sigmas')

    # Obtain the bias' standard deviation in all directions, in m/s^2 and rad/s.
    prior_bias_sigmas = rospy.get_param('prior_bias_sigmas')

    # Obtain the accelerometer noise standard deviations in all directions, in m/s^2.
    accel_sigmas = rospy.get_param('accel_sigmas')

    # Obtain the gyroscope noise standard deviations in all directions, in rad/s.
    gyro_sigmas = rospy.get_param('gyro_sigmas')

    # Obtain the integration noise standard deviations in all directions.
    integration_sigmas = rospy.get_param('integration_sigmas')

    # Obtain the initital estimates for the prior poses, velocities, and IMU biases.
    prior_pose_estimate = rospy.get_param('prior_pose_estimate')
    prior_vel_estimate = rospy.get_param('prior_vel_estimate')
    prior_bias_estimate = rospy.get_param('prior_bias_estimate')

    # Parse the config parameters to be GTSAM appropriate objects.
    parsed_config_params = _parse_config_parameters(prior_pose_estimate,
                                                    prior_vel_estimate,
                                                    prior_bias_estimate,
                                                    prior_pose_sigmas,
                                                    prior_vel_sigmas,
                                                    prior_bias_sigmas,
                                                    accel_sigmas,
                                                    gyro_sigmas,
                                                    integration_sigmas)

    # Create the initial factor graph and associated parameters for setup.
    return _create_factor_graph_and_params(graph, initial_estimates, *parsed_config_params)


def preintegrate_measurement(high_state_msg: HighState,
                             delta_t: float,
                             pim: gtsam.PreintegratedImuMeasurements,
                             n_gravity: np.ndarray):
    """Extracts IMU measurement from HighState message and
    preintegrate the IMU measurement.
    Args:
        high_state_msg: The current HighState message from the A1.
        delta_t: The amount of time between the current and previous measurement.
        pim: The gtsam object to perform the IMU preintegration.
        n_gravity: A numpy array representing the gravity vector.
    Returns:
        pim: The gtsam object after performing preintegration.
    """
    measured_accel = np.array(high_state_msg.imu.accelerometer)
    measured_omega = np.array(high_state_msg.imu.gyroscope)
    pim.integrateMeasurement(measured_accel, measured_omega, delta_t)
    return pim


def add_IMU_factor(i: int,
                   j: int,
                   pim: gtsam.PreintegratedImuMeasurements,
                   navstate: gtsam.NavState,
                   graph: gtsam.NonlinearFactorGraph,
                   initial_estimates: gtsam.Values,
                   results: gtsam.Values):
    """Adds an IMU factor to the factor graph and
    relevant initial estimates.
    Args:
        i: The previous state index of the IMU factor.
        j: The subsequent state index of the IMU factor.
        pim: The PIM used to create the IMU factor.
        navstate: The navigation state used to predict the next state.
        graph: The current factor graph before adding the IMU factor.
        initial_estimates: The current set of initial estimates.
        results: The current estimates of the trajectory.
    Returns:
        graph: An updated factor graph with the IMU factor.
        initial_estimates: Updates initial estimates.
        pim: The pim with reset integration.
    """
    # Add the IMU factor to the factor graph.
    imu_factor = gtsam.ImuFactor(
        X(i), V(i), X(j), V(j), B(0), pim)
    graph.push_back(imu_factor)

    # Initialize the estimates for the pose and velocity by integrating from the previous state.
    predicted_state = pim.predict(navstate, results.atConstantBias(B(0)))
    initial_estimates.insert(X(j), predicted_state.pose())
    initial_estimates.insert(V(j), predicted_state.velocity())

    # Reset the integration of the pim.
    pim.resetIntegration()

    return graph, initial_estimates, pim
