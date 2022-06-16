import unittest

import a1_slam
import gtsam
import numpy as np
from gtsam.symbol_shorthand import X
from gtsam.utils.test_case import GtsamTestCase
from sensors import LIDAR_2D_helpers
from sensor_msgs.msg import LaserScan


class TestIMU(GtsamTestCase):

    def setUp(self):
        # Create default prior factors and covariances.
        self.setup_pose_prior = gtsam.PriorFactorPose2(
            X(0),
            gtsam.Pose2(),
            gtsam.noiseModel.Isotropic.Sigma(3, 1e-10)
        )
    
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
                [1e-2, 1e-2, 1e-3]
            )
        )

        # Generate the actual prior factors and covariances.
        actual = LIDAR_2D_helpers._parse_config_parameters(
                    prior_pose_estimate,
                    prior_pose_sigmas,
                    icp_noise_sigmas)
        
        self.gtsamAssertEquals(actual[0], expected_pose_prior)

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
            self.setup_pose_prior
        )

        self.gtsamAssertEquals(actual[0], expected_graph)
        self.gtsamAssertEquals(actual[1], expected_inits)
    
    def test_preprocess_measurement(self):
        """Test preintegrating several measurements."""
        scan = LaserScan()
        angles = [0, 45, 90, 135, 180, 225, 270, 315]
        for i in range(len(angles)):
            scan.ranges[i] = 1.0 if angles[i] % 10 == 0 else np.sqrt(2)
        expected = np.array([
            [-1.0, -1.0, 0.0, 1.0, 1.0, 1.0, 0.0, -1.0],
            [0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0, 1.0],
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        ])
        actual = LIDAR_2D_helpers.preprocess_measurement(scan)
        np.testing.assert_array_equal(expected, actual)

    def test_add_lidar_factor(self):
        """Test adding an IMU factor to the graph."""

        # Initialize the necessary parameters for adding the IMU factor.
        graph = gtsam.NonlinearFactorGraph()
        graph.add(self.setup_pose_prior)
        initial_estimates = gtsam.Values()
        initial_estimates.insert(X(0), gtsam.Pose2())
        scan_a = np.array([[0.0, 0.5, 0.5], [0.0, 0.0, 1.0], [1.0, 1.0, 1.0]])
        scan_b = np.array([[0.1, 0.6, 0.6], [0.0, 0.0, 1.0], [1.0, 1.0, 1.0]])

        graph, initial_estimates = LIDAR_2D_helpers.add_lidar_factor(
            X(0),
            X(1),
            scan_a,
            scan_b,
            graph,
            initial_estimates
        )

        # Instantiate the iSAM2 parameters to create the iSAM2 object.
        isam = gtsam.ISAM2(gtsam.ISAM2Params())

        # Perform the incremental update using iSAM2.
        isam.update(graph, initial_estimates)
        actual = isam.calculateEstimate()

        expected = gtsam.Values()
        expected.insert(X(0), gtsam.Pose2())
        expected.insert(X(1), gtsam.Pose2(0.1, 0, 0))
        self.gtsamAssertEquals(actual, expected, tol=5e-3)

if __name__ == '__main__':
    unittest.main()