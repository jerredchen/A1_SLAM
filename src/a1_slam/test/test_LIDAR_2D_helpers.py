import unittest

import a1_slam
import gtsam
import numpy as np
from gtsam.symbol_shorthand import X
from gtsam.utils.test_case import GtsamTestCase
from sensors import LIDAR_2D_helpers
from sensor_msgs.msg import LaserScan


class TestLIDAR(GtsamTestCase):

    def setUp(self):
        # Create default prior factors and covariances.
        self.setup_pose_prior = gtsam.PriorFactorPose2(
            X(0),
            gtsam.Pose2(),
            gtsam.noiseModel.Diagonal.Sigmas([1e-10, 1e-10, 1e-10])
        )
        self.icp_noise = gtsam.noiseModel.Diagonal.Sigmas([
            1e-3, 1e-3, 1e-3
        ])
    
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
        actual = LIDAR_2D_helpers._parse_config_parameters(
                    prior_pose_estimate,
                    prior_pose_sigmas,
                    icp_noise_sigmas)
        
        self.gtsamAssertEquals(actual[0], expected_pose_prior)
        self.gtsamAssertEquals(actual[1], expected_icp_noise)

    def test_create_factor_graph_and_params(self):
        """Test the creation of the factor graph and additional setup."""
        
        # Generate the expected factor graph with prior factors.
        expected_graph = gtsam.NonlinearFactorGraph()
        expected_graph.add(self.setup_pose_prior)

        # Generate the expected initial estimates.
        expected_inits = gtsam.Values()
        expected_inits.insert(X(0), gtsam.Pose2())

        # Generate the actual factor graph and initial estimates.
        actual = LIDAR_2D_helpers._create_factor_graph_and_params(
            gtsam.NonlinearFactorGraph(),
            gtsam.Values(),
            self.setup_pose_prior
        )

        self.gtsamAssertEquals(actual[0], expected_graph)
        self.gtsamAssertEquals(actual[1], expected_inits)
    
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
            [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0],
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        ])
        actual = LIDAR_2D_helpers.preprocess_measurement(scan)
        np.testing.assert_array_almost_equal(expected, actual, decimal=1e-5)

    def test_add_lidar_factor(self):
        """Test adding an IMU factor to the graph."""

        # Initialize the necessary parameters for adding the IMU factor.
        graph = gtsam.NonlinearFactorGraph()
        graph.add(self.setup_pose_prior)
        initial_estimates = gtsam.Values()
        initial_estimates.insert(X(0), gtsam.Pose2())

        scan_a = np.array([[0.0, 0.5, 0.5], [0.0, 0.0, 1.0], [1.0, 1.0, 1.0]])
        scan_b = np.array([[0.1, 0.6, 0.6], [0.0, 0.0, 1.0], [1.0, 1.0, 1.0]])

        # Instantiate the iSAM2 parameters to create the iSAM2 object.
        isam = gtsam.ISAM2(gtsam.ISAM2Params())

        # Perform the first incremental update using iSAM2 and clear the graph and initial estimate.
        isam.update(graph, initial_estimates)
        results = isam.calculateEstimate()
        graph = gtsam.NonlinearFactorGraph()
        initial_estimates.clear()

        graph, initial_estimates = LIDAR_2D_helpers.add_lidar_factor(
            0,
            1,
            scan_a,
            scan_b,
            self.icp_noise,
            graph,
            initial_estimates,
            results,
            registration="vanilla"
        )

        # Perform the actual incremental update using iSAM2.
        isam.update(graph, initial_estimates)
        actual = isam.calculateEstimate()

        expected = gtsam.Values()
        expected.insert(X(0), gtsam.Pose2())
        expected.insert(X(1), gtsam.Pose2(-0.1, 0, 0))
        self.gtsamAssertEquals(actual, expected, tol=5e-3)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('a1_slam', 'test_lidar_helpers', TestLIDAR)