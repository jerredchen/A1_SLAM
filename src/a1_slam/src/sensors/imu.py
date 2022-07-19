#!/usr/bin/env python

"""
Class for processing IMU measurements.
"""
import rospy

import gtsam
import numpy as np
from gtsam.symbol_shorthand import B, V, X
from optimization.optimizer import Optimizer
from sensor_msgs.msg import Imu
from unitree_legged_msgs.msg import HighState


class ImuWrapper():

    def __init__(self, optimizer: Optimizer):

        # Instantiate trajectory and optimizer related attributes.
        self.state_index = 1
        self.optimizer = optimizer

        # Instantiate IMU related attributes.
        self.timestamp = 0
        self.seen_measurements = 0

    ###################### Callbacks ######################

    def imu_unitree_callback(self, msg: HighState, imu_only=False):
        """Perform IMU preintegration whenever receiving an IMU
        measurement.
        Args:
            msg:        A HighState message to extract IMU measurements from.
            imu_only:   Whether IMU is the only sensor used.
        """
        if self.timestamp == 0:
            self.timestamp = msg.header.stamp.to_sec()
            self.seen_measurements += 1
            return
        current_timestamp = msg.header.stamp.to_sec()
        self.preintegrate_measurement(msg, current_timestamp - self.timestamp)
        self.timestamp = current_timestamp
        self.seen_measurements += 1
        factor_rate = rospy.get_param('/imu/imu_factor_rate')
        if imu_only and self.seen_measurements == factor_rate:
            # Calculate the NavState based on the optimized results.
            pose = self.optimizer.results.atPose3(X(self.state_index))
            velocity = self.optimizer.results.atVector(V(self.state_index))
            navstate = gtsam.NavState(pose, velocity)

            # Create an IMU factor and the initial estimates.
            imu_factor, _, predicted_navstate = self.create_IMU_factor(
                navstate)
            pose_estimate = (
                X(self.state_index + 1),
                predicted_navstate.pose()
            )
            vel_estimate = (
                V(self.state_index + 1),
                predicted_navstate.velocity()
            )

            # Add the factor and initial estimates to the optimizer.
            self.optimizer.add_factor(imu_factor, [pose_estimate, vel_estimate])
            self.optimizer.optimize()
            self.state_index += 1
            self.seen_measurements = 0

    def imu_ros_callback(self, msg: Imu, imu_only=False):
        """Perform IMU preintegration whenever receiving an IMU
        measurement.
        Args:
            msg:        A ROS Imu message.
            imu_only:   Whether IMU is the only sensor used.
        """
        if self.timestamp == 0:
            self.timestamp = msg.header.stamp.to_sec()
            self.seen_measurements += 1
            return
        current_timestamp = msg.header.stamp.to_sec()
        self.preintegrate_measurement(msg, current_timestamp - self.timestamp)
        self.timestamp = current_timestamp
        self.seen_measurements += 1
        factor_rate = rospy.get_param('/imu/imu_factor_rate')
        if imu_only and self.seen_measurements == factor_rate:
            # Calculate the NavState based on the optimized results.
            pose = self.optimizer.results.atPose3(X(self.state_index))
            velocity = self.optimizer.results.atVector(V(self.state_index))
            navstate = gtsam.NavState(pose, velocity)

            # Create an IMU factor and the initial estimates.
            imu_factor, _, predicted_navstate = self.create_IMU_factor(
                navstate)
            pose_estimate = (
                X(self.state_index + 1),
                predicted_navstate.pose()
            )
            vel_estimate = (
                V(self.state_index + 1),
                predicted_navstate.velocity()
            )

            # Add the factor and initial estimates to the optimizer.
            self.optimizer.add_factor(imu_factor, [pose_estimate, vel_estimate])
            self.optimizer.optimize()
            self.state_index += 1
            self.seen_measurements = 0

    ###################### Callable functions ######################

    def get_aTb_estimate(self, a, b):
        """Estimate aTb from IMU which is used as an initial estimate for ICP.
        Args:
            a:          The state index associated with the previous time step.
            b:          The state index associated with the current time step.
        Returns:
            aTb:        The gtsam.Pose3 relative pose used for an ICP initial estimate.
        """
        # Calculate the NavState based on the optimized results.
        pose = self.optimizer.results.atPose3(X(a))
        velocity = self.optimizer.results.atVector(V(a))
        navstate = gtsam.NavState(pose, velocity)

        # Create an IMU factor and the initial estimates.
        factor, aTb, predicted_navstate = self.create_IMU_factor(
            a, b, navstate)
        pose_estimate = (X(b), predicted_navstate.pose())
        vel_estimate = (V(b), predicted_navstate.velocity())

        # Add the factor and initial estimates to the optimizer.
        self.optimizer.add_factor(factor, [pose_estimate, vel_estimate])
        self.optimizer.optimize()
        return aTb

    ###################### Helper functions ######################

    def preintegrate_measurement(self,
                                 msg,
                                 delta_t: float):
        """Extracts IMU measurement from HighState message and
        preintegrate the IMU measurement.
        Args:
            msg:     The current IMU message from the A1.
            delta_t: The amount of time between the current and previous measurement.
        """
        if type(msg) == HighState:
            measured_accel = np.array(msg.imu.accelerometer)
            measured_omega = np.array(msg.imu.gyroscope)
        elif type(msg) == Imu:
            measured_accel, measured_omega = np.zeros((3,)), np.zeros((3,))
            measured_accel[0] = msg.linear_acceleration.x
            measured_accel[1] = msg.linear_acceleration.y
            measured_accel[2] = msg.linear_acceleration.z
            measured_omega[0] = msg.angular_velocity.x
            measured_omega[1] = msg.angular_velocity.y
            measured_omega[2] = msg.angular_velocity.z
        self.pim.integrateMeasurement(measured_accel, measured_omega, delta_t)

    def create_IMU_factor(self, a, b, navstate):
        """Creates an IMU factor to the factor graph and
        relevant initial estimates.
        Args:
            a:        The state index associated with the previous time step.
            b:        The state index associated with the current time step.
            navstate: The NavState at the current time step, used to predict the next time step.
        Returns:
            factor:             An IMU factor to be added to the factor graph.
            aTb:                A gtsam.Pose3 of the relative pose between poses a and b.
            predicted_navstate: The predicted NavState at the next time step.
        """
        # Add the IMU factor to the factor graph.
        imu_factor = gtsam.ImuFactor(X(a), V(a), X(b), V(b), B(0), self.pim)

        # Initialize the estimates for the pose and velocity by integrating from the previous state.
        predicted_navstate = self.pim.predict(
            navstate, self.optimizer.results.atConstantBias(B(0)))

        # Calculate the relative pose between poses a and b.
        pose_a, pose_b = navstate.pose(), predicted_navstate.pose()
        aTb = pose_a.between(pose_b)

        # Reset the integration of the pim, and reinitialize the NavState.
        self.pim.resetIntegration()

        return imu_factor, aTb, predicted_navstate

    ###################### Initialization and preprocessing ######################

    def initialize_params(self):
        """Initialize Imu parameters based on the set rosparams."""

        # Obtain the accelerometer noise standard deviations in all directions, in m/s^2.
        accel_sigmas = rospy.get_param('/imu/accel_sigmas')

        # Obtain the gyroscope noise standard deviations in all directions, in rad/s.
        gyro_sigmas = rospy.get_param('/imu/gyro_sigmas')

        # Obtain the integration noise standard deviations in all directions.
        integration_sigmas = rospy.get_param('/imu/integration_sigmas')

        # Instantiate covariance matrices according to IMU standard deviations.
        accel_noise_cov = np.diag(np.square(accel_sigmas))
        gyro_noise_cov = np.diag(np.square(gyro_sigmas))
        integration_noise_cov = np.diag(np.square(integration_sigmas))

        # Parse the bias parameters and create a GTSAM bias factor.
        prior_bias_estimate = rospy.get_param('/imu/prior_bias_estimate')
        accel_bias, gyro_bias = prior_bias_estimate[:3], prior_bias_estimate[3:]
        prior_bias_estimate = gtsam.imuBias.ConstantBias(
            accel_bias, gyro_bias)

        # Instantiate the PIM parameters to create the PIM object.
        pim_parameters = gtsam.PreintegrationParams.MakeSharedU()
        pim_parameters.setAccelerometerCovariance(accel_noise_cov)
        pim_parameters.setGyroscopeCovariance(gyro_noise_cov)
        pim_parameters.setIntegrationCovariance(integration_noise_cov)
        self.pim = gtsam.PreintegratedImuMeasurements(
            pim_parameters, prior_bias_estimate)
