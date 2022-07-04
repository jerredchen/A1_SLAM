import gtsam
import numpy as np
from gtsam.symbol_shorthand import X, V, B
from gtsam.utils.test_case import GtsamTestCase
from nodes.ImuNode import ImuNode
from unitree_legged_msgs.msg import HighState

class TestImuNode(GtsamTestCase):

    def setUp(self):
        self.imu_node = ImuNode()
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
        pim_params = gtsam.PreintegrationParams.MakeSharedU()
        self.imu_node.pim = gtsam.PreintegratedImuMeasurements(pim_params)

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
            gtsam.Pose3(gtsam.Rot3.Ypr(np.pi/2, np.pi/4, 0),
                        gtsam.Point3(1, 2, 3)),
            gtsam.noiseModel.Diagonal.Sigmas(
                np.concatenate(
                    (np.deg2rad(prior_pose_sigmas[3:]), prior_pose_sigmas[:3]))
            )
        )
        expected_vel_prior = gtsam.PriorFactorVector(
            V(0),
            np.array(prior_vel_estimate),
            gtsam.noiseModel.Diagonal.Sigmas(prior_vel_sigmas)
        )
        expected_bias_prior = gtsam.PriorFactorConstantBias(
            B(0),
            gtsam.imuBias.ConstantBias(
                prior_bias_estimate[:3], prior_bias_estimate[3:]),
            gtsam.noiseModel.Diagonal.Sigmas(prior_bias_sigmas)
        )
        expected_accel_cov = np.array(
            [[1e-6, 0.0, 0.0], [0.0, 1e-8, 0.0], [0, 0, 1e-10]])
        expected_gyro_cov = np.array(
            [[1e-6, 0.0, 0.0], [0.0, 1e-8, 0.0], [0, 0, 1e-10]])
        expected_int_cov = np.array(
            [[1e-6, 0.0, 0.0], [0.0, 1e-8, 0.0], [0, 0, 1e-10]])

        # Generate the actual prior factors and covariances.
        actual = self.imu_node.parse_config_parameters(
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

    def test_preintegrate(self):
        """Test preintegrating several measurements."""

        highstate = HighState()
        highstate.imu.accelerometer = [1.0, 0.0, 0.0]
        for _ in range(50):
            self.imu_node.preintegrate_measurement(highstate, 0.02)

        np.testing.assert_array_almost_equal(
            self.imu_node.pim.deltaPij(), [0.5, 0, 0], decimal=1e-10)

    def test_add_IMU_factor(self):
        """Test adding an IMU factor to the graph."""

        # Initialize the necessary parameters for adding the IMU factor.
        graph = gtsam.NonlinearFactorGraph()
        graph.add(self.setup_pose_prior)
        graph.add(self.setup_vel_prior)
        graph.add(self.setup_bias_prior)
        initial_estimates = gtsam.Values()
        initial_estimates.insert(X(0), gtsam.Pose3())
        initial_estimates.insert(V(0), np.zeros((3,)))
        initial_estimates.insert(B(0), gtsam.imuBias.ConstantBias())
        highstate = HighState()
        highstate.imu.accelerometer = [1.0, 0.0, 9.81]
        for _ in range(50):
            self.imu_node.preintegrate_measurement(highstate, 0.02)

        # Instantiate the iSAM2 parameters to create the iSAM2 object.
        isam = gtsam.ISAM2(gtsam.ISAM2Params())

        # Perform the preliminary update and clear the graph and initial estimates.
        isam.update(graph, initial_estimates)
        self.imu_node.results = isam.calculateEstimate()
        graph = gtsam.NonlinearFactorGraph()
        initial_estimates.clear()

        navstate = gtsam.NavState()
        imu_factor, predicted_state = self.imu_node.add_IMU_factor(0, 1, navstate)
        graph.add(imu_factor)
        initial_estimates.insert(X(1), predicted_state.pose())
        initial_estimates.insert(V(1), predicted_state.velocity())

        # Perform the actual incremental update using iSAM2.
        isam.update(graph, initial_estimates)
        actual = isam.calculateEstimate()

        expected = gtsam.Values()
        for i in range(2):
            expected.insert(X(i), gtsam.Pose3(
                gtsam.Rot3(), gtsam.Point3(0.5*i, 0.0, 0.0)))
            expected.insert(V(i), gtsam.Point3(i, 0, 0))
        expected.insert(B(0), gtsam.imuBias.ConstantBias())
        self.gtsamAssertEquals(actual, expected)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('a1_slam', 'test_imu', TestImuNode)
