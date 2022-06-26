"""
Unit tests for ICP.
"""

import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase
from registration import icp

class TestICP(GtsamTestCase):

  def setUp(self):
    self.scan_a = np.array([[0, 0, 1], [0, 1, 0]])
    self.scan_b = np.array([[1, 1, 2], [0, 1, 0]])
    self.scan_c = np.array([[0, 20, 10], [0, 10, 20]])
    self.scan_d = np.array([[10, 30, 20], [10, 20, 30]])
    self.scan_e = np.array([[0, -1, 0], [0, 0, 1]])
    self.scan_f = np.array([[0, 0, -1], [0, -1, 0]])
    self.triangle = np.array([[0.0, 1.0, 1.0], [0.0, 0.0, 0.5]])

  def test_transform_scan(self):
    actual_scan = icp.transform_scan(self.scan_a, gtsam.Pose2(1,0,0))
    np.testing.assert_allclose(actual_scan, self.scan_b)

  def test_estimate_transform_SVD_1(self):
    actual_transform = icp.estimate_transform(self.scan_a, self.scan_b)
    self.gtsamAssertEquals(actual_transform, gtsam.Pose2(-1,0,0), tol=1e-6)

  def test_estimate_transform_SVD_3(self):
    actual_transform = icp.estimate_transform(self.scan_c, self.scan_d)
    self.gtsamAssertEquals(actual_transform, gtsam.Pose2(-10,-10,0), tol=1e-6)

  def test_estimate_transform_SVD_4(self):
    expected_transform = gtsam.Pose2(0, 0, -np.pi/2)
    actual_transform = icp.estimate_transform(self.scan_a, self.scan_e)
    self.gtsamAssertEquals(actual_transform, expected_transform, tol=1e-6)

  def test_estimate_transform_SVD_5(self):
    expected_transform = gtsam.Pose2(0, 0, -np.pi)
    actual_transform = icp.estimate_transform(self.scan_a, self.scan_f)
    self.gtsamAssertEquals(actual_transform, expected_transform, tol=1e-6)

  def test_icp_1(self):
    """
    Vertical line shift 1 units to the right.
    """
    scan_a = np.array([[0, 0, 0, 0],[0, 1, 2, 3]])
    scan_b = np.array([[1, 1, 1, 1],[0, 1, 2, 3]])
    aTb = gtsam.Pose2(-1, 0, 0)
    actual_transform = icp.icp(scan_a, scan_b)
    self.gtsamAssertEquals(actual_transform, aTb, tol=1e-1)

  def test_icp_2(self):
    """
    Triangle shifts one unit to the left and downward.
    """
    expected_transform = gtsam.Pose2(0.1, 0.1, 0)
    actual_transform = icp.icp(self.triangle + 0.1, self.triangle)
    self.gtsamAssertEquals(actual_transform, expected_transform, tol=1e-1)

if __name__ == "__main__":
  unittest.main()