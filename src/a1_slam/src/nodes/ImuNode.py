#!/usr/bin/env python

"""
Node for processing IMU measurements.
"""
import rospy

import gtsam
import numpy as np
from a1_slam.srv import GtsamResults
from gtsam.symbol_shorthand import B, V, X
from unitree_legged_msgs.msg import HighState


class ImuNode():

    def __init__(self):
        self.request_optimizer = None

        self.pim = None
        self.state_index = 0

    def parse_config_parameters(self,
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

        return prior_pose_factor, prior_vel_factor, prior_bias_factor, \
            accel_noise_cov, gyro_noise_cov, integration_noise_cov

    def create_pim(self,
                   bias: gtsam.imuBias.ConstantBias,
                   accel_noise: np.ndarray,
                   gyro_noise: np.ndarray,
                   integration_noise: np.ndarray):
        """Create a factor graph and the necessary parameters for optimization.

        Args:
            bias:               The initial biases associated with the IMU.
            accel_noise:        A numpy array representing the covariance of the accelerometer.
            gyro_noise:         A numpy array representing the covariance of the gyroscope.
            integration_noise:  A numpy array representing the covariance of the IMU integration.
        """

        # Instantiate the PIM parameters to create the PIM object.
        pim_parameters = gtsam.PreintegrationParams.MakeSharedU()
        pim_parameters.setAccelerometerCovariance(accel_noise)
        pim_parameters.setGyroscopeCovariance(gyro_noise)
        pim_parameters.setIntegrationCovariance(integration_noise)
        self.pim = gtsam.PreintegratedImuMeasurements(
            pim_parameters, bias)

    def imu_callback(self, msg: HighState):
        pass

    def preintegrate_measurement(self,
                                 high_state_msg: HighState,
                                 delta_t: float):
        """Extracts IMU measurement from HighState message and
        preintegrate the IMU measurement.
        Args:
            high_state_msg:     The current HighState message from the A1.
            delta_t:            The amount of time between the current and previous measurement.
        """
        measured_accel = np.array(high_state_msg.imu.accelerometer)
        measured_omega = np.array(high_state_msg.imu.gyroscope)
        self.pim.integrateMeasurement(measured_accel, measured_omega, delta_t)

    def add_IMU_factor(self,
                       i: int,
                       j: int,
                       navstate: gtsam.NavState):
        """Adds an IMU factor to the factor graph and
        relevant initial estimates.
        Args:
            i:          The previous state index of the IMU factor.
            j:          The subsequent state index of the IMU factor.
            navstate:   The navigation state used to predict the next state.
        Returns:
            factor: An IMU factor to be added to the factor graph.
            predicted_state: The navigation state used as an initial estimate.
        """
        # Add the IMU factor to the factor graph.
        imu_factor = gtsam.ImuFactor(
            X(i), V(i), X(j), V(j), B(0), self.pim)

        # Initialize the estimates for the pose and velocity by integrating from the previous state.
        predicted_state = self.pim.predict(
            navstate, self.results.atConstantBias(B(0)))

        # Reset the integration of the pim.
        self.pim.resetIntegration()

        return imu_factor, predicted_state

    def launch_imu_node(self):

        rospy.init_node('imu_node', anonymous=True)

        # Instantiate service for optimizer.
        rospy.wait_for_service('optimizer_service')
        self.request_optimizer = rospy.ServiceProxy(
            'optimizer_service', GtsamResults)

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
        parsed_config_params = self.parse_config_parameters(
            prior_pose_estimate,
            prior_vel_estimate,
            prior_bias_estimate,
            prior_pose_sigmas,
            prior_vel_sigmas,
            prior_bias_sigmas,
            accel_sigmas,
            gyro_sigmas,
            integration_sigmas
        )
        prior_pose_factor = parsed_config_params[0]
        prior_vel_factor = parsed_config_params[1]
        prior_bias_factor = parsed_config_params[2]
        accel_noise_cov = parsed_config_params[3]
        gyro_noise_cov = parsed_config_params[4]
        integration_noise_cov = parsed_config_params[5]

        # Request results from adding prior to the factor graph.
        serialized_factor = prior_pose_factor.serialize()
        serialized_estimate = prior_pose_factor.prior().serialize()
        response = self.request_optimizer(
            "PriorFactorPose3",
            serialized_factor,
            serialized_estimate
        )
        received_results = gtsam.Values()
        received_results.deserialize(response.results)
        self.results = received_results
        self.state_index += 1

        # Instantiate the PIM parameters to create the PIM object.
        pim_parameters = gtsam.PreintegrationParams.MakeSharedU()
        pim_parameters.setAccelerometerCovariance(accel_noise_cov)
        pim_parameters.setGyroscopeCovariance(gyro_noise_cov)
        pim_parameters.setIntegrationCovariance(integration_noise_cov)
        self.pim = gtsam.PreintegratedImuMeasurements(
            pim_parameters, prior_bias_factor.prior())

        topic = rospy.get_param('/imu/topic')
        rospy.Subscriber(topic, HighState, self.imu_callback)

        rospy.spin()


if __name__ == "__main__":
    try:
        imu_node = ImuNode()
        imu_node.launch_imu_node()
    except rospy.ROSInterruptException:
        pass
