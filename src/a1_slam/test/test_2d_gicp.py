"""
Unit tests for ICP.
"""

import unittest

import gtsam
import numpy as np
import pygicp
from gtsam.utils.test_case import GtsamTestCase

class TestICP(GtsamTestCase):

    def setUp(self):
        num = 75
        length = 1
        left = np.vstack((np.zeros((num,)), np.linspace(0, length, num)))
        right = np.vstack((np.linspace(0, length, num),
                          np.linspace(length, 0, num)))
        bottom = np.vstack((np.linspace(0, length, num), np.zeros((num,))))
        triangle = np.hstack((left, right, bottom))
        self.triangle = np.vstack((triangle, np.ones((triangle.shape[1],))))

    def test_icp_1(self):
        """Triangle shifts 0.1 unit to the left and downward."""
        aTb_2D = gtsam.Pose2(0.1, 0.1, 0)
        expected = gtsam.Pose3(aTb_2D)
        aTb_3D = pygicp.align_points(
            expected.transformFrom(self.triangle).T,
            self.triangle.T,
            method="GICP",
            max_correspondence_distance=0.5,
            k_correspondences=3,
            num_threads=4
        )
        actual = gtsam.Pose3(aTb_3D)
        self.gtsamAssertEquals(actual, expected, tol=1e-3)

    def test_icp_2(self):
        """Triangle rotated 15 degrees."""
        aTb_2D = gtsam.Pose2(0, 0, np.pi/12)
        expected = gtsam.Pose3(aTb_2D)
        aTb_3D = pygicp.align_points(
            expected.transformFrom(self.triangle).T,
            self.triangle.T,
            method="GICP",
            max_correspondence_distance=0.5,
            k_correspondences=3,
            num_threads=4
        )
        actual = gtsam.Pose3(aTb_3D)
        self.gtsamAssertEquals(actual, expected, tol=1e-3)



if __name__ == "__main__":
    unittest.main()
