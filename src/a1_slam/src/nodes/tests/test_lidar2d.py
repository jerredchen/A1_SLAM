import gtsam
import numpy as np
from gtsam.symbol_shorthand import X
from gtsam.utils.test_case import GtsamTestCase
from nodes.Lidar2DNode import Lidar2DNode
from sensor_msgs.msg import LaserScan


class TestLIDAR2D(GtsamTestCase):

    def setUp(self):
        self.lidar_node = Lidar2DNode()
        # Create default prior factors and covariances.
        self.setup_pose_prior = gtsam.PriorFactorPose2(
            X(0),
            gtsam.Pose2(),
            gtsam.noiseModel.Diagonal.Sigmas([1e-10, 1e-10, 1e-10])
        )
        self.icp_noise = gtsam.noiseModel.Diagonal.Sigmas([
            1e-3, 1e-3, 1e-3
        ])
        self.lidar_node.icp_noise_model = self.icp_noise

    def test_parse_config_parameters(self):
        """Test the parsing of the config parameters."""

        # Instantiate toy config values.
        prior_pose_estimate = [1, 2, 0]
        prior_pose_sigmas = [1e-2, 1e-2, 1e-3]
        icp_noise_sigmas = [1e-2, 1e-2, 1e-3]

        # Generate expected prior factors and covariances.
        expected_pose_prior = gtsam.PriorFactorPose2(
            X(0),
            gtsam.Pose2(1, 2, 0),
            gtsam.noiseModel.Diagonal.Sigmas(
                [1e-2, 1e-2, np.deg2rad(1e-3)]
            )
        )

        expected_icp_noise = gtsam.noiseModel.Diagonal.Sigmas(
            [1e-2, 1e-2, np.deg2rad(1e-3)]
        )

        # Generate the actual prior factors and covariances.
        actual = self.lidar_node.parse_config_parameters(
            prior_pose_estimate,
            prior_pose_sigmas,
            icp_noise_sigmas)

        self.gtsamAssertEquals(actual[0], expected_pose_prior)
        self.gtsamAssertEquals(actual[1], expected_icp_noise)

    def test_preprocess_measurement(self):
        """Test preprocessing the scan ranges."""
        scan = LaserScan()
        scan.angle_increment = -np.pi/4
        scan.angle_min, scan.angle_max = np.pi, -np.pi
        angles = [0, 45, 90, 135, 180, 225, 270, 315]
        for i in range(len(angles)):
            scan.ranges.append(1.0 if i % 2 == 0 else np.sqrt(2))
        expected = np.array([
            [-1.0, -1.0, 0.0, 1.0, 1.0, 1.0, 0.0, -1.0],
            [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0]
        ])
        actual = self.lidar_node.preprocess_measurement(scan)
        np.testing.assert_array_almost_equal(expected, actual, decimal=1e-5)

    def test_create_lidar_factor(self):
        """Test creating a LIDAR odometry factor and initial estimate."""

        self.lidar_node.results.insert(X(0), gtsam.Pose2())

        scan_a = np.array([[0.0, 0.5, 0.5], [0.0, 0.0, 1.0]])
        scan_b = np.array([[0.1, 0.6, 0.6], [0.0, 0.0, 1.0]])

        actual_factor, actual_estimate = self.lidar_node.create_lidar_factor(
            0,
            1,
            scan_a,
            scan_b,
            registration="vanilla"
        )

        expected_factor = gtsam.BetweenFactorPose2(
            X(0),
            X(1),
            gtsam.Pose2(-0.1, 0, 0),
            self.icp_noise
        )
        expected_estimate = gtsam.Pose2()
        self.gtsamAssertEquals(actual_factor, expected_factor, tol=5e-3)
        self.gtsamAssertEquals(actual_estimate, expected_estimate, tol=5e-3)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('a1_slam', 'test_lidar', TestLIDAR2D)
