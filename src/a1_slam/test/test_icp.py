"""
Unit tests for ICP.
"""

import unittest

import numpy as np

import gtsam
from registration import icp
from gtsam.utils.test_case import GtsamTestCase
import time

class TestICP(GtsamTestCase):

    def setUp(self):
        self.scan_a = np.array([[0, 0, 1], [0, 1, 0]])
        self.scan_b = np.array([[1, 1, 2], [0, 1, 0]])
        self.triangle = np.array([[0.0, 1.0, 1.0], [0.0, 0.0, 0.5]])

        num = 75
        length = 1
        left = np.vstack((np.zeros((num,)), np.linspace(0, length, num)))
        right = np.vstack((np.linspace(0, length, num),
                          np.linspace(length, 0, num)))
        bottom = np.vstack((np.linspace(0, length, num), np.zeros((num,))))
        self.large_triangle = np.hstack((left, right, bottom))

    def test_estimate_normals_1(self):
        """Estimate normals for a vertical line."""
        scan = np.array([[-1, -1, -1, -1, -1], [0, 1, 2, 3, 4]])
        expected = np.array([[1, 1, 1, 1, 1], [0, 0, 0, 0, 0]])
        actual = icp.estimate_normals(scan, k_nearest=3)
        np.testing.assert_allclose(actual, expected)

    def test_estimate_normals_2(self):
        """Estimate normals for a 45 degree angled line."""
        scan = np.array([[0, 1, 2, 3, 4], [4, 3, 2, 1, 0]])
        expected = np.array([[-np.sqrt(2)]*5, [-np.sqrt(2)]*5]) / 2.0
        actual = icp.estimate_normals(scan, k_nearest=3)
        np.testing.assert_allclose(actual, expected)

    def test_transform_scan(self):
        actual_scan = icp.transform_scan(self.scan_a, gtsam.Pose2(1, 0, 0))
        np.testing.assert_allclose(actual_scan, self.scan_b)

    def test_estimate_transform_1(self):
        actual_transform = icp.estimate_transform(
            self.scan_a,
            self.scan_b
        )
        self.gtsamAssertEquals(
            actual_transform,
            gtsam.Pose2(-1, 0, 0),
            tol=1e-6
        )

    def test_estimate_transform_2(self):
        scan_a_rotated = np.array([[0, -1, 0], [0, 0, 1]])
        expected_transform = gtsam.Pose2(0, 0, -np.pi/2)
        actual_transform = icp.estimate_transform(
            self.scan_a,
            scan_a_rotated
        )
        self.gtsamAssertEquals(actual_transform, expected_transform, tol=1e-6)

    def test_estimate_transform_3(self):
        scan = np.array([[0, 0, 0, 0, 0], [0, 1, 2, 3, 4]])
        scan_shifted = np.array([[1, 1, 1, 1, 1], [0, 1, 2, 3, 4]])
        normals = np.array([[1, 1, 1, 1, 1], [0, 0, 0, 0, 0]])
        expected = gtsam.Pose2(-1, 0, 0)
        actual = icp.estimate_transform(scan, scan_shifted, normals)
        self.gtsamAssertEquals(actual, expected, tol=1e-6)

    def test_icp_1(self):
        """Vertical line shift 1 units to the right."""
        scan_a = np.array([[0, 0, 0, 0], [0, 1, 2, 3]])
        scan_b = np.array([[1, 1, 1, 1], [0, 1, 2, 3]])
        aTb = gtsam.Pose2(-1, 0, 0)
        actual_transform = icp.icp(scan_a, scan_b)
        self.gtsamAssertEquals(actual_transform, aTb, tol=1e-2)

    def test_icp_2(self):
        """Triangle shifts 0.1 unit to the left and downward."""
        expected_transform = gtsam.Pose2(0.1, 0.1, 0)
        actual_transform = icp.icp(
            self.triangle + 0.1,
            self.triangle
        )
        self.gtsamAssertEquals(actual_transform, expected_transform, tol=1e-2)

    def test_icp_3(self):
        """Triangle shifts 0.1 units left and downward."""
        expected = gtsam.Pose2(0.1, 0.1, 0)
        normals = icp.estimate_normals(
            self.large_triangle + 0.1,
            k_nearest=2
        )
        actual = icp.icp(
            self.large_triangle + 0.1,
            self.large_triangle,
            init_aTb=gtsam.Pose2(),
            normals_a=normals
        )
        self.gtsamAssertEquals(actual, expected, tol=1e-2)

    def test_icp_4(self):
        """Triangle rotated 15 degrees."""
        expected = gtsam.Pose2(0, 0, np.pi/12)
        normals = icp.estimate_normals(
            expected.transformFrom(self.large_triangle),
            k_nearest=2
        )

        actual = icp.icp(
            expected.transformFrom(self.large_triangle),
            self.large_triangle,
            init_aTb=gtsam.Pose2(),
            normals_a=normals
        )
        self.gtsamAssertEquals(actual, expected, tol=1e-2)


if __name__ == "__main__":
    unittest.main()
