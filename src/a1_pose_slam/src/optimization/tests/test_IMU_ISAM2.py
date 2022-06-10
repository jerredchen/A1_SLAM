import unittest

import gtsam
import numpy as np
from gtsam.symbol_shorthand import B, V, X
from gtsam.utils.test_case import GtsamTestCase
from optimization import A1_IMU_ISAM2
from sensor_msgs.msg import Imu


class TestIMU(GtsamTestCase):

    def setUp(self):
        # Create default prior factors and covariances.
        self.setup_pose_prior = gtsam.PriorFactorPose3(
            X(0),
            gtsam.Pose3(),
            gtsam.noiseModel.Isotropic.Sigma(6, 1e-10)
        )
        self.setup_vel_prior = gtsam.PriorFactorVector(
            V(0),
            np.zeros(3),
            gtsam.noiseModel.Isotropic.Sigma(3, 1e-10)
        )
        self.setup_bias_prior = gtsam.PriorFactorConstantBias(
            B(0),
            gtsam.imuBias.ConstantBias(),
            gtsam.noiseModel.Isotropic.Sigma(6, 1e-3)
        )
        self.setup_accel_noise = np.diag([1e-3, 1e-3, 1e-3])
        self.setup_gyro_noise = np.diag([1e-5, 1e-5, 1e-5])
        self.setup_integration_noise = np.diag([1e-5, 1e-5, 1e-5])

    def generate_stationary_data(self):
        """Generate IMU measurements when stationary."""
        for i in range(5):
            imu = Imu()
            imu.header.stamp.secs = i
            imu.linear_acceleration.x = 0
            imu.linear_acceleration.y = 0
            imu.linear_acceleration.z = 9.81
            imu.angular_velocity.x = 0
            imu.angular_velocity.y = 0
            imu.angular_velocity.z = 0
            yield ('/IMU', imu)

    def generate_preint_gravity_data(self):
        """Generate higher frequency IMU measurements when stationary."""
        for i in range(250):
            imu = Imu()
            imu.header.stamp.secs = i / 50
            imu.linear_acceleration.x = 0
            imu.linear_acceleration.y = 0
            imu.linear_acceleration.z = 9.81
            imu.angular_velocity.x = 0
            imu.angular_velocity.y = 0
            imu.angular_velocity.z = 0
            yield ('/IMU', imu)

    def generate_biased_gravity_data(self):
        """Generate biased IMU measurements when stationary."""
        for i in range(250):
            imu = Imu()
            imu.header.stamp.secs = i / 50
            imu.linear_acceleration.x = -0.1
            imu.linear_acceleration.y = 0.1
            imu.linear_acceleration.z = 9.81
            imu.angular_velocity.x = 0
            imu.angular_velocity.y = 0
            imu.angular_velocity.z = 0
            yield ('/IMU', imu)

    def generate_constant_accel_data(self):
        """Generate IMU measurements with constant acceleration in x-direction."""
        for i in range(250):
            imu = Imu()
            imu.header.stamp.secs = i / 50
            imu.linear_acceleration.x = 1.0
            imu.linear_acceleration.y = 0.0
            imu.linear_acceleration.z = 9.81
            imu.angular_velocity.x = 0
            imu.angular_velocity.y = 0
            imu.angular_velocity.z = 0
            yield ('/IMU', imu)

    def generate_constant_yaw_data(self):
        """Generate IMU measurements with constant acceleration in x-direction."""
        for i in range(250):
            imu = Imu()
            imu.header.stamp.secs = i / 50
            imu.linear_acceleration.x = 0.0
            imu.linear_acceleration.y = 0.0
            imu.linear_acceleration.z = 9.81
            imu.angular_velocity.x = 0
            imu.angular_velocity.y = 0
            imu.angular_velocity.z = np.pi/12
            yield ('/IMU', imu)
    
    def test_parse_config_parameters(self):
        """Test the parsing of the config parameters."""

        # Instantiate toy config values.
        prior_pose_estimate = [1, 2, 3, 0, 45, 90]
        prior_vel_estimate = [4, 5, 6]
        prior_bias_estimate = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
        prior_pose_sigmas = [1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3]
        prior_vel_sigmas = [1e-2, 1e-3, 1e-4]
        prior_bias_sigmas = [1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3]
        accel_sigmas = [1e-3, 1e-4, 1e-5]
        gyro_sigmas = [1e-3, 1e-4, 1e-5]
        integration_sigmas = [1e-3, 1e-4, 1e-5]

        # Generate expected prior factors and covariances.
        expected_pose_prior = gtsam.PriorFactorPose3(
            X(0),
            gtsam.Pose3(gtsam.Rot3.Ypr(np.pi/2, np.pi/4, 0), gtsam.Point3(1, 2, 3)),
            gtsam.noiseModel.Diagonal.Sigmas(
                np.concatenate((np.deg2rad(prior_pose_sigmas[3:]), prior_pose_sigmas[:3]))
            )
        )
        expected_vel_prior = gtsam.PriorFactorVector(
            V(0),
            np.array(prior_vel_estimate),
            gtsam.noiseModel.Diagonal.Sigmas(prior_vel_sigmas)
        )
        expected_bias_prior = gtsam.PriorFactorConstantBias(
            B(0),
            gtsam.imuBias.ConstantBias(prior_bias_estimate[:3], prior_bias_estimate[3:]),
            gtsam.noiseModel.Diagonal.Sigmas(prior_bias_sigmas)
        )
        expected_accel_cov = np.array([[1e-6, 0.0, 0.0], [0.0, 1e-8, 0.0], [0, 0, 1e-10]])
        expected_gyro_cov = np.array([[1e-6, 0.0, 0.0], [0.0, 1e-8, 0.0], [0, 0, 1e-10]])
        expected_int_cov = np.array([[1e-6, 0.0, 0.0], [0.0, 1e-8, 0.0], [0, 0, 1e-10]])

        # Generate the actual prior factors and covariances.
        actual = A1_IMU_ISAM2.parse_config_parameters(
                    prior_pose_estimate,
                    prior_vel_estimate,
                    prior_bias_estimate,
                    prior_pose_sigmas,
                    prior_vel_sigmas,
                    prior_bias_sigmas,
                    accel_sigmas,
                    gyro_sigmas,
                    integration_sigmas)
        
        self.gtsamAssertEquals(actual[0], expected_pose_prior)
        self.gtsamAssertEquals(actual[1], expected_vel_prior)
        self.gtsamAssertEquals(actual[2], expected_bias_prior)
        self.gtsamAssertEquals(actual[3], expected_accel_cov)
        self.gtsamAssertEquals(actual[4], expected_gyro_cov)
        self.gtsamAssertEquals(actual[5], expected_int_cov)

    def test_create_factor_graph_and_params(self):
        """Test the creation of the factor graph and additional setup."""
        
        # Generate the expected factor graph with prior factors.
        expected_graph = gtsam.NonlinearFactorGraph()
        expected_graph.add(self.setup_pose_prior)
        expected_graph.add(self.setup_vel_prior)
        expected_graph.add(self.setup_bias_prior)

        # Generate the expected initial estimates.
        expected_inits = gtsam.Values()
        expected_inits.insert(X(0), gtsam.Pose3())
        expected_inits.insert(V(0), np.zeros(3))
        expected_inits.insert(B(0), gtsam.imuBias.ConstantBias())

        # Generate the actual factor graph and initial estimates.
        actual = A1_IMU_ISAM2.create_factor_graph_and_params(
            self.setup_pose_prior,
            self.setup_vel_prior,
            self.setup_bias_prior,
            self.setup_accel_noise,
            self.setup_gyro_noise,
            self.setup_integration_noise
        )

        self.gtsamAssertEquals(actual[0], expected_graph)
        self.gtsamAssertEquals(actual[1], expected_inits)

    def test_only_gravity(self):
        """Test IMU factor graph with stationary IMU measurements."""

        data = self.generate_stationary_data()

        # Generate the actual pose, velocity, and bias estimate values.
        graph, initial_estimate, isam, pim = A1_IMU_ISAM2.create_factor_graph_and_params(
            self.setup_pose_prior,
            self.setup_vel_prior,
            self.setup_bias_prior,
            self.setup_accel_noise,
            self.setup_gyro_noise,
            self.setup_integration_noise
        )
        actual = A1_IMU_ISAM2.optimize_trajectory(
            data,
            '/IMU',
            graph,
            initial_estimate,
            isam,
            pim,
            imu_factor_rate=1
        )

        # Generate the expected pose, velocity, and bias values.
        expected = gtsam.Values()
        for i in range(5):
            expected.insert(X(i), gtsam.Pose3())
            expected.insert(V(i), gtsam.Point3(0, 0, 0))
        expected.insert(B(0), gtsam.imuBias.ConstantBias())
        self.gtsamAssertEquals(actual, expected, 1e-3)

    def test_preint_gravity(self):
        """Test IMU factor graph with preintegration."""

        data = self.generate_preint_gravity_data()

        # Generate the actual pose, velocity, and bias estimate values.
        graph, initial_estimate, isam, pim = A1_IMU_ISAM2.create_factor_graph_and_params(
            self.setup_pose_prior,
            self.setup_vel_prior,
            self.setup_bias_prior,
            self.setup_accel_noise,
            self.setup_gyro_noise,
            self.setup_integration_noise
        )
        actual = A1_IMU_ISAM2.optimize_trajectory(
            data,
            '/IMU',
            graph,
            initial_estimate,
            isam,
            pim,
            imu_factor_rate=50
        )

        # Generate the expected pose, velocity, and bias estimate values.
        expected = gtsam.Values()
        for i in range(5):
            expected.insert(X(i), gtsam.Pose3())
            expected.insert(V(i), gtsam.Point3(0, 0, 0))
        expected.insert(B(0), gtsam.imuBias.ConstantBias())
        self.gtsamAssertEquals(actual, expected, 1e-3)


    def test_bias_correction(self):
        """Test the IMU factor graph bias correction."""

        data = self.generate_biased_gravity_data()

        bias_prior = gtsam.PriorFactorConstantBias(
            B(0),
            gtsam.imuBias.ConstantBias(gtsam.Point3(-0.1, 0.1, 0), gtsam.Point3(0, 0, 0)),
            gtsam.noiseModel.Isotropic.Sigma(6, 1e-3)
        )

        # Generate the actual pose, velocity, and bias estimate values.
        graph, initial_estimate, isam, pim = A1_IMU_ISAM2.create_factor_graph_and_params(
            self.setup_pose_prior,
            self.setup_vel_prior,
            bias_prior,
            self.setup_accel_noise,
            self.setup_gyro_noise,
            self.setup_integration_noise
        )
        actual = A1_IMU_ISAM2.optimize_trajectory(
            data,
            '/IMU',
            graph,
            initial_estimate,
            isam,
            pim,
            imu_factor_rate=50
        )

        # Generate the expected pose, velocity, and bias estimate values.
        expected = gtsam.Values()
        for i in range(5):
            expected.insert(X(i), gtsam.Pose3())
            expected.insert(V(i), gtsam.Point3(0, 0, 0))
        expected.insert(B(0), gtsam.imuBias.ConstantBias(gtsam.Point3(-0.1, 0.1, 0),
                                                         gtsam.Point3(0, 0, 0)))
        self.gtsamAssertEquals(actual, expected, 1e-3)

    def test_const_vel(self):
        """Test the IMU factor graph with constant velocities."""

        data = self.generate_stationary_data()
        velocity_prior = gtsam.PriorFactorVector(
            V(0),
            np.array([1.0, 0.0, 0.0]),
            gtsam.noiseModel.Isotropic.Sigma(3, 1e-3)
        )

        # Generate the actual pose, velocity, and bias estimate values.
        graph, initial_estimate, isam, pim = A1_IMU_ISAM2.create_factor_graph_and_params(
            self.setup_pose_prior,
            velocity_prior,
            self.setup_bias_prior,
            self.setup_accel_noise,
            self.setup_gyro_noise,
            self.setup_integration_noise
        )
        actual = A1_IMU_ISAM2.optimize_trajectory(
            data,
            '/IMU',
            graph,
            initial_estimate,
            isam,
            pim,
            imu_factor_rate=1
        )

        # Generate the expected pose, velocity, and bias estimate values.
        expected = gtsam.Values()
        for i in range(5):
            expected.insert(X(i), gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(i, 0.0, 0.0)))
            expected.insert(V(i), gtsam.Point3(1.0, 0.0, 0.0))
        expected.insert(B(0), gtsam.imuBias.ConstantBias())
        self.gtsamAssertEquals(actual, expected, 1e-3)

    def test_const_accel(self):
        """Test the IMU factor graph with constant acceleration."""

        data = self.generate_constant_accel_data()

        # Generate the actual pose, velocity, and bias estimate values.
        graph, initial_estimate, isam, pim = A1_IMU_ISAM2.create_factor_graph_and_params(
            self.setup_pose_prior,
            self.setup_vel_prior,
            self.setup_bias_prior,
            self.setup_accel_noise,
            self.setup_gyro_noise,
            self.setup_integration_noise
        )
        actual = A1_IMU_ISAM2.optimize_trajectory(
            data,
            '/IMU',
            graph,
            initial_estimate,
            isam,
            pim,
            imu_factor_rate=50
        )

        # Generate the expected pose, velocity, and bias estimate values.
        expected = gtsam.Values()
        for i in range(5):
            expected.insert(X(i), gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0.5*(i**2), 0.0, 0.0)))
            expected.insert(V(i), gtsam.Point3(i, 0, 0))
        expected.insert(B(0), gtsam.imuBias.ConstantBias())
        self.gtsamAssertEquals(actual, expected, 1e-3)

    def test_constant_ang_vel(self):
        """Test the IMU factor graph with constant acceleration."""

        data = self.generate_constant_yaw_data()

        # Generate the actual pose, velocity, and bias estimate values.
        graph, initial_estimate, isam, pim = A1_IMU_ISAM2.create_factor_graph_and_params(
            self.setup_pose_prior,
            self.setup_vel_prior,
            self.setup_bias_prior,
            self.setup_accel_noise,
            self.setup_gyro_noise,
            self.setup_integration_noise
        )
        actual = A1_IMU_ISAM2.optimize_trajectory(
            data,
            '/IMU',
            graph,
            initial_estimate,
            isam,
            pim,
            imu_factor_rate=50
        )

        # Generate the expected pose, velocity, and bias estimate values.
        expected = gtsam.Values()
        for i in range(5):
            expected.insert(X(i), gtsam.Pose3(gtsam.Rot3.Rz(i*np.pi/12), gtsam.Point3(0.0, 0.0, 0.0)))
            expected.insert(V(i), gtsam.Point3(0, 0, 0))
        expected.insert(B(0), gtsam.imuBias.ConstantBias())
        self.gtsamAssertEquals(actual, expected, 1e-3)

if __name__ == "__main__":
    unittest.main()
