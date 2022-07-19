import gtsam
import numpy as np
import time
from gtsam.symbol_shorthand import X
from gtsam.utils.test_case import GtsamTestCase
from sensor_msgs.msg import LaserScan

from optimization.optimizer import Optimizer
from sensors.lidar2D import Lidar2DWrapper


class TestLIDAR2D(GtsamTestCase):

    def setUp(self):
        
        self.optimizer = Optimizer()
        self.lidar2d = Lidar2DWrapper(self.optimizer)
        # Create default prior factors and covariances.
        self.setup_pose_prior = gtsam.PriorFactorPose3(
            X(0),
            gtsam.Pose3(),
            gtsam.noiseModel.Diagonal.Sigmas([1e-10]*6)
        )
        self.icp_noise = gtsam.noiseModel.Diagonal.Sigmas([
            1e-20, 1e-20, 1e-4, 1e-3, 1e-3, 1e-20
        ])
        self.lidar2d.icp_noise_model = self.icp_noise

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
        actual = self.lidar2d.preprocess_measurement(scan)
        np.testing.assert_array_almost_equal(expected, actual, decimal=1e-5)

    def test_create_lidar_factor_1(self):
        """Test creating a LIDAR odometry factor and initial estimate."""
        # Reset the optimizer attributes.
        self.optimizer.results.clear()
        self.optimizer.isam = gtsam.ISAM2(gtsam.ISAM2Params())
        self.optimizer.results.insert(X(0), gtsam.Pose3())

        scan_a = np.array([[0.0, 0.5, 0.5], [0.0, 0.0, 1.0]])
        scan_b = np.array([[0.1, 0.6, 0.6], [0.0, 0.0, 1.0]])

        actual_factor, actual_estimate = self.lidar2d.create_lidar_factor(
            0,
            1,
            scan_a,
            scan_b
        )

        expected_factor = gtsam.BetweenFactorPose3(
            X(0),
            X(1),
            gtsam.Pose3(gtsam.Pose2(-0.1, 0, 0)),
            self.icp_noise
        )
        expected_estimate = gtsam.Pose3(gtsam.Pose2(-0.1, 0, 0))
        self.gtsamAssertEquals(actual_factor, expected_factor, tol=5e-3)
        self.gtsamAssertEquals(actual_estimate, expected_estimate, tol=5e-3)

    def test_create_lidar_factor_2(self):
        """Test creating a LIDAR odometry factor and initial estimate."""
        # Reset the optimizer attributes.
        self.optimizer.results.clear()
        self.optimizer.isam = gtsam.ISAM2(gtsam.ISAM2Params())
        self.optimizer.results.insert(X(0), gtsam.Pose3())

        scan_a = np.array([[0.0, 0.5, 0.5], [0.0, 0.0, 1.0]])
        scan_b = np.array([[0.1, 0.6, 0.6], [0.0, 0.0, 1.0]])

        actual_factor, actual_estimate = self.lidar2d.create_lidar_factor(
            0,
            1,
            scan_a,
            scan_b,
            gtsam.Pose3(gtsam.Pose2(-0.05, 0, 0))
        )

        expected_factor = gtsam.BetweenFactorPose3(
            X(0),
            X(1),
            gtsam.Pose3(gtsam.Pose2(-0.1, 0, 0)),
            self.icp_noise
        )
        expected_estimate = gtsam.Pose3(gtsam.Pose2(-0.1, 0, 0))
        self.gtsamAssertEquals(actual_factor, expected_factor, tol=5e-3)
        self.gtsamAssertEquals(actual_estimate, expected_estimate, tol=5e-3)

    def test_create_skip_connection(self):
        """Test creating a single skip connection."""
        # Reset the optimizer attributes.
        self.optimizer.results.clear()
        self.optimizer.isam = gtsam.ISAM2(gtsam.ISAM2Params())
        submap = []
        scan = np.array([[0.0, 0.5, 0.5], [0.0, 0.0, 1.0]])
        for i in range(3):
            new_scan = scan + np.array([[0.1*i], [0.0]])
            submap.append(new_scan)
            pose = gtsam.Pose3(gtsam.Pose2(-0.1*i, 0, 0))
            self.optimizer.results.insert(X(i), pose)

        # Create the skip connection between states 0 and 2.
        self.lidar2d.create_skip_connection(
            submap,
            0,
            2
        )
        expected_factor = gtsam.BetweenFactorPose3(
            X(0),
            X(2),
            gtsam.Pose3(gtsam.Pose2(-0.2, 0, 0)),
            self.icp_noise
        )
        queue = self.optimizer.factor_queue
        self.assertEqual(len(queue), 1)
        actual_factor, _ = queue[0]
        self.gtsamAssertEquals(actual_factor, expected_factor, tol=5e-3)
    
    def test_create_skip_connections(self):
        """Test creating multiple skip connections. """
        # Reset the optimizer attributes.
        self.optimizer.results.clear()
        self.optimizer.isam = gtsam.ISAM2(gtsam.ISAM2Params())

        # Instantiate and add necessary factors to optimizer.
        current_index = 9
        scan = np.array([[0.0, 0.5, 0.5], [0.0, 0.0, 1.0]])
        prior = gtsam.PriorFactorPose3(
            X(0),
            gtsam.Pose3(),
            self.icp_noise
        )
        self.lidar2d.submap_scans.append(scan)
        self.optimizer.add_factor(prior, (X(0), gtsam.Pose3()))
        for i in range(current_index):
            aTb_factor = gtsam.BetweenFactorPose3(
                X(i),
                X(i+1),
                gtsam.Pose3(gtsam.Pose2(-0.1, 0, 0)),
                self.icp_noise
            )
            wTb = gtsam.Pose3(gtsam.Pose2(-0.1*(i+1), 0, 0))
            self.optimizer.add_factor(aTb_factor, (X(i+1), wTb))
            new_scan = scan + np.array([[0.1*(i+1)], [0.0]])
            self.lidar2d.submap_scans.append(new_scan)

        # Perform optimization.
        self.optimizer.optimize()
        submap = self.lidar2d.submap_scans

        # Create skip connections.
        self.lidar2d.create_skip_connections(submap, current_index)
        queue = self.optimizer.factor_queue
        self.assertEqual(len(queue), current_index - 1)

        for i in range(len(queue)):
            expected_factor = gtsam.BetweenFactorPose3(
                X(current_index - 2 - i),
                X(current_index),
                gtsam.Pose3(gtsam.Pose2(-0.2-(0.1*i), 0, 0)),
                self.icp_noise
            )
            actual_factor, _ = queue[i]
            self.gtsamAssertEquals(actual_factor, expected_factor, tol=1e-7)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('a1_slam', 'test_lidar', TestLIDAR2D)
