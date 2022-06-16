import unittest

import gtsam
import numpy as np
import sgdicp2D
from gtsam.utils.test_case import GtsamTestCase

class TestBayesianICP(GtsamTestCase):

    def setUp(self):
        num_points = 60
        wall_length = 1.5
        wall_points = np.linspace(0, wall_length, num_points)

        left_wall = np.vstack((np.zeros(num_points), wall_points, np.ones(num_points)))
        right_wall = np.vstack((wall_length*np.ones(num_points), wall_points, np.ones(num_points)))
        lower_wall = np.vstack((wall_points, np.zeros(num_points), np.ones(num_points)))
        upper_wall = np.vstack((wall_points, wall_length*np.ones(num_points), np.ones(num_points)))
        self.square = np.hstack((left_wall, upper_wall, right_wall, lower_wall))

        tri_bottom = np.vstack((np.linspace(0, 1, num_points), np.zeros(num_points), np.ones(num_points)))
        tri_side = np.vstack((np.zeros(num_points), np.linspace(0, 1, num_points), np.ones(num_points)))
        tri_hypo = np.vstack((np.linspace(0, 1, num_points), np.linspace(1, 0, num_points), np.ones(num_points)))
        self.triangle = np.hstack((tri_bottom, tri_side, tri_hypo))

    def test_square_trans(self):
        scan1 = self.square
        scan2 = self.square + 0.1

        expected = gtsam.Pose2(0.1, 0.1, 0.0)
        burn_in = 100
        samples = sgdicp2D.bayesian_icp(scan1, scan2, [0.11, 0.09, 0.0], [0.1, 0.1, 0.0], [1e-2, 1e-2, 1e-2])
        mean_tf = np.mean(samples[burn_in:,:], axis=0)
        actual = gtsam.Pose2(*mean_tf)
        self.gtsamAssertEquals(actual, expected, tol=1e-2)

    def test_square_trans_no_prior(self):
        scan1 = self.square
        scan2 = self.square + 0.1

        expected = gtsam.Pose2(0.1, 0.1, 0.0)
        burn_in = 100
        samples = sgdicp2D.bayesian_icp(scan1, scan2, [0.11, 0.09, 0.0], [0.0, 0.0, 0.0], [100, 100, 100])
        mean_tf = np.mean(samples[burn_in:,:], axis=0)
        actual = gtsam.Pose2(*mean_tf)
        self.gtsamAssertEquals(actual, expected, tol=1e-2)

    def test_square_rot(self):
        scan1 = self.square
        scan2 = gtsam.Pose2(0, 0, np.pi/6).matrix() @ self.square

        expected = gtsam.Pose2(0.0, 0.0, np.pi/6)
        burn_in = 100
        samples = sgdicp2D.bayesian_icp(scan1, scan2, [0.0, 0.0, np.pi/7], [0.0, 0.0, np.pi/6], [1e-2, 1e-2, 1e-2])
        mean_tf = np.mean(samples[burn_in:,:], axis=0)
        actual = gtsam.Pose2(*mean_tf)
        self.gtsamAssertEquals(actual, expected, tol=1e-2)
    
    def test_square_tf_no_prior(self):
        scan1 = self.square
        scan2 = gtsam.Pose2(0.1, 0.1, np.pi/6).matrix() @ self.square

        expected = gtsam.Pose2(0.1, 0.1, np.pi/6)
        burn_in = 100
        samples = sgdicp2D.bayesian_icp(scan1, scan2, [0.11, 0.11, np.pi/7], [0.0, 0.0, 0.0], [100, 100, 100])
        mean_tf = np.mean(samples[burn_in:,:], axis=0)
        actual = gtsam.Pose2(*mean_tf)
        self.gtsamAssertEquals(actual, expected, tol=1e-2)
    
    def test_triangle_no_prior(self):
        scan1 = self.triangle
        scan2 = gtsam.Pose2(0.1, 0.1, np.pi/4).matrix() @ scan1

        expected = gtsam.Pose2(0.1, 0.1, np.pi/4)
        samples = sgdicp2D.bayesian_icp(scan1, scan2, [0.11, 0.09, 9*np.pi/40], [0.0, 0.0, 0.0], [100, 100, 100])
        burn_in = 100
        mean_tf = np.mean(samples[burn_in:,:], axis=0)
        actual = gtsam.Pose2(*mean_tf)
        self.gtsamAssertEquals(actual, expected, tol=1e-2) 
        

if __name__ == "__main__":
  unittest.main()
