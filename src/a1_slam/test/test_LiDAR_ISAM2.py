import unittest

import numpy as np

import gtsam
# import icp_line
from registration import vanilla_ICP
import sgdicp2D
from collections import deque
from gtsam.utils.test_case import GtsamTestCase

class TestLiDAR(GtsamTestCase):

    def setUp(self):
        self.result = gtsam.Values()
        self.poses = [gtsam.Pose2(i/10, 0, 0) for i in range(10)]
        for i, pose in enumerate(self.poses):
            self.result.insert(i, pose)

        num_points = 60
        wall_length = 1.5
        wall_points = np.linspace(0, wall_length, num_points)

        left_wall = np.vstack(
            (np.zeros(num_points), wall_points, np.ones(num_points)))
        right_wall = np.vstack(
            (wall_length*np.ones(num_points), wall_points, np.ones(num_points)))
        lower_wall = np.vstack(
            (wall_points, np.zeros(num_points), np.ones(num_points)))
        upper_wall = np.vstack(
            (wall_points, wall_length*np.ones(num_points), np.ones(num_points)))
        self.square = np.hstack(
            (left_wall, upper_wall, right_wall, lower_wall))

    def generate_points(self):
        for i in range(len(self.poses)):
            yield self.square.copy() - np.array([i/10, 0, 0]).reshape(-1, 1)

    def optimize_trajectory(self,
                            points,
                            skip_steps=2,
                            prior_x_pose_sigma=1e-5,
                            prior_y_pose_sigma=1e-5,
                            prior_heading_sigma=1e-4):
        """Incrementally optimize the A1's trajectory from LiDAR measurements.

        Args:
            points: A generator which yields a numpy array of shape (3,n) representing a scan.
            skip_steps: The size of the difference in indices for creating skip connections.
            prior_x_pose_sigma: The standard deviation of the noise model for the prior pose's x-direction, in m.
            prior_y_pose_sigma: The standard deviation of the noise model for the prior pose's y-direction, in m.
            prior_heading_sigma: The standard deviation of the noise model for the prior pose's heading, in deg.
        """

        # Instantiate the factor graph and its initial estimates container.
        graph = gtsam.NonlinearFactorGraph()
        initial_estimate = gtsam.Values()

        # Instantiate the iSAM2 parameters to create the iSAM2 object.
        parameters = gtsam.ISAM2Params()
        parameters.setRelinearizeThreshold(0.1)
        isam = gtsam.ISAM2(parameters)

        # Declare noise models for the prior pose and the associated noise with ICP.
        PRIOR_POSE_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([prior_x_pose_sigma, prior_y_pose_sigma, prior_heading_sigma * np.pi / 180]))
        ICP_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([1e-1, 1e-1, 1e-1]))

        # Add the prior factor to the factor graph with its associated initial estimate.
        graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(), PRIOR_POSE_NOISE))
        initial_estimate.insert(0, gtsam.Pose2())
        isam.update(graph, initial_estimate)
        initial_estimate.clear()
        result = isam.calculateEstimate()

        # Initialize the transforms needed before performing any optimization.
        aTb = gtsam.Pose2()
        stored_scans = deque([], maxlen=skip_steps)
        scan_prev = gtsam.Pose2()

        # for k, bag_info in enumerate(measurements):
        for k, point_scan in enumerate(points):

            # Convert the scan from a 1D array of ranges to an array of 2D points in the local pose frame.
            scan_b = point_scan

            if k == 0:
                stored_scans.append(scan_b)
                continue

            for i in range(min(k, skip_steps)):
                print(
                    f"*******************scans {k}, {k-(i+1)}*******************")
                if i == 0:
                    init_estimate = scan_prev
                else:
                    wTb = result.atPose2(k)
                    wTa = result.atPose2(k - (i + 1))
                    init_estimate = wTa.between(wTb)
                # Estimate the transform between two consecutive scan measurements.
                aTb = vanilla_ICP.icp(scan_b, stored_scans[-(i + 1)], initial_transform=init_estimate)
                # Add an odometry factor between two poses and its initial estimate from dead reckoning.
                graph.add(gtsam.BetweenFactorPose2(
                    k - (i + 1), k, aTb, ICP_NOISE))
                if i == 0:
                    initialized_odom = result.atPose2(k - 1).compose(aTb)
                    initial_estimate.insert(k, initialized_odom)
                    scan_prev = aTb

                # Perform an iSAM2 incremental update.
                isam.update(graph, initial_estimate)
                result = isam.calculateEstimate()

                # Clear the graph and initial estimates.
                graph = gtsam.NonlinearFactorGraph()
                initial_estimate.clear()

            stored_scans.append(scan_b)
        return result

    def optimize_bips_trajectory(self,
                                 points,
                                 skip_steps=2,
                                 prior_x_pose_sigma=1e-5,
                                 prior_y_pose_sigma=1e-5,
                                 prior_heading_sigma=1e-4):
        """Incrementally optimize the A1's trajectory from LiDAR measurements.

        Args:
            points: A generator which yields a numpy array of shape (3,n) representing a scan.
            skip_steps: The size of the difference in indices for creating skip connections.
            prior_x_pose_sigma: The standard deviation of the noise model for the prior pose's x-direction, in m.
            prior_y_pose_sigma: The standard deviation of the noise model for the prior pose's y-direction, in m.
            prior_heading_sigma: The standard deviation of the noise model for the prior pose's heading, in deg.
        """

        # Instantiate the factor graph and its initial estimates container.
        graph = gtsam.NonlinearFactorGraph()
        initial_estimate = gtsam.Values()

        # Instantiate the iSAM2 parameters to create the iSAM2 object.
        parameters = gtsam.ISAM2Params()
        parameters.setRelinearizeThreshold(0.1)
        isam = gtsam.ISAM2(parameters)

        # Declare noise models for the prior pose and the associated noise with ICP.
        PRIOR_POSE_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([prior_x_pose_sigma, prior_y_pose_sigma, prior_heading_sigma * np.pi / 180]))
        ICP_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([1e-1, 1e-1, 1e-1]))

        # Add the prior factor to the factor graph with its associated initial estimate.
        graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(), PRIOR_POSE_NOISE))
        initial_estimate.insert(0, gtsam.Pose2())
        isam.update(graph, initial_estimate)
        initial_estimate.clear()
        result = isam.calculateEstimate()

        # Initialize the transforms needed before performing any optimization.
        aTb = gtsam.Pose2()
        stored_scans = deque([], maxlen=skip_steps)
        scan_prev = gtsam.Pose2(5e-3, 5e-3, 5e-2)

        # for k, bag_info in enumerate(measurements):
        for k, point_scan in enumerate(points):

            # Convert the scan from a 1D array of ranges to an array of 2D points in the local pose frame.
            scan_b = point_scan

            if k == 0:
                stored_scans.append(scan_b)
                continue

            for i in range(min(k, skip_steps)):
                print(
                    f"*******************scans {k}, {k-(i+1)}*******************")
                if i == 0:
                    init_estimate = [
                        scan_prev.x(), scan_prev.y(), scan_prev.theta()]
                else:
                    wTb = result.atPose2(k)
                    wTa = result.atPose2(k - (i + 1))
                    aTb_estimate = wTa.between(wTb)
                    init_estimate = [
                        aTb_estimate.x(), aTb_estimate.y(), aTb_estimate.theta()]
                # Estimate the transform between two consecutive scan measurements.
                # aTb, _, skip = icp_line.icp(scan_b, stored_scans[-(i + 1)], initial_transform=init_estimate)
                samples = sgdicp2D.bayesian_icp(
                    scan_b, stored_scans[-(i + 1)], init_estimate, [0, 0, 0], [100, 100, 100])
                posterior_mean = np.mean(samples[100:, :], axis=0)
                aTb = gtsam.Pose2(*posterior_mean)
                # Add an odometry factor between two poses and its initial estimate from dead reckoning.
                graph.add(gtsam.BetweenFactorPose2(
                    k - (i + 1), k, aTb, ICP_NOISE))
                if i == 0:
                    initialized_odom = result.atPose2(k - 1).compose(aTb)
                    initial_estimate.insert(k, initialized_odom)
                    scan_prev = aTb

                # Perform an iSAM2 incremental update.
                isam.update(graph, initial_estimate)
                result = isam.calculateEstimate()

                # Clear the graph and initial estimates.
                graph = gtsam.NonlinearFactorGraph()
                initial_estimate.clear()

            stored_scans.append(scan_b)
        return result

    def test_factor_graph(self):
        points = self.generate_points()

        expected = self.result
        actual = self.optimize_trajectory(points, skip_steps=6)

        self.gtsamAssertEquals(actual, expected, 1e-2)

    # def test_bips_factor_graph(self):
    #     points = self.generate_points()

    #     expected = self.result
    #     actual = self.optimize_bips_trajectory(points, skip_steps=6)

    #     self.gtsamAssertEquals(actual, expected, 1e-2)


if __name__ == "__main__":
    unittest.main()
