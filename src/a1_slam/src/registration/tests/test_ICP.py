"""
Unit tests for Personal ICP.
"""

import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase
from registration import vanilla_ICP

class TestICP(GtsamTestCase):

  def setUp(self):
    self.scan_a = np.array([[0, 0, 1], [0, 1, 0], [1, 1, 1,]])
    self.scan_b = np.array([[1, 1, 2], [0, 1, 0], [1, 1, 1,]])
    self.scan_c = np.array([[1], [1], [1]])
    self.scan_d = np.array([[2], [1], [1]])
    self.scan_f = np.array([[0, 20, 10], [0, 10, 20], [1, 1, 1,]])
    self.scan_g = np.array([[10, 30, 20], [10, 20, 30], [1, 1, 1,]])
    self.scan_h = np.array([[0.0, 1.0, 1.0], [0.0, 0.0, 0.5], [1, 1, 1,]])
    self.scan_i = np.array([[0, -1, 0], [0, 0, 1], [1, 1, 1,]])
    self.scan_j = np.array([[0, 0, -1], [0, -1, 0], [1, 1, 1,]])

  def test_transform_scan_1(self):
    actual_scan = vanilla_ICP.transform_scan(self.scan_a, gtsam.Pose2(1,0,0))
    np.testing.assert_allclose(actual_scan, self.scan_b)

  def test_transform_scan_2(self):
    actual_scan = vanilla_ICP.transform_scan(self.scan_f, gtsam.Pose2(10,10,0))
    np.testing.assert_allclose(actual_scan, self.scan_g)

  def test_estimate_transform_SVD_1(self):
    actual_transform = vanilla_ICP.estimate_transform(self.scan_a, self.scan_b)
    self.gtsamAssertEquals(actual_transform, gtsam.Pose2(1,0,0), tol=1e-6)

  def test_estimate_transform_SVD_3(self):
    actual_transform = vanilla_ICP.estimate_transform(self.scan_f, self.scan_g)
    self.gtsamAssertEquals(actual_transform, gtsam.Pose2(10,10,0), tol=1e-6)
  
  def test_estimate_transform_SVD_4(self):
    expected_transform = gtsam.Pose2(0, 0, np.pi/2)
    actual_transform = vanilla_ICP.estimate_transform(self.scan_a, self.scan_i)
    self.gtsamAssertEquals(actual_transform, expected_transform, tol=1e-6)

  def test_estimate_transform_SVD_5(self):
    expected_transform = gtsam.Pose2(0, 0, np.pi)
    actual_transform = vanilla_ICP.estimate_transform(self.scan_a, self.scan_j)
    self.gtsamAssertEquals(actual_transform, expected_transform, tol=1e-6)
  
  def test_icp_1(self):
    """
    Triangle shifts one unit to the right.
    Triangle points order is left, top, right.
    """
    expected_transform = gtsam.Pose2(-1, 0, 0)
    actual_transform = vanilla_ICP.icp(self.scan_b, self.scan_a)
    np.testing.assert_almost_equal(actual_transform.x(), expected_transform.x(), decimal=1)
  
  def test_icp_2(self):
    """
    Vertical line shift 1 units to the right.
    """
    scan_a = np.array([[0, 0, 0, 0],[0, 1, 2, 3], [1, 1, 1, 1]])
    scan_b = np.array([[1, 1, 1, 1],[0, 1, 2, 3], [1, 1, 1, 1]])
    aTb = gtsam.Pose2(1, 0, 0)
    actual_transform = vanilla_ICP.icp(scan_a, scan_b)
    self.gtsamAssertEquals(actual_transform, aTb, tol=1e-1)

  def test_icp_3(self):
    """
    Triangle shifts one unit to the left and downward.
    """
    expected_transform = gtsam.Pose2(-1, -1, 0)
    actual_transform = vanilla_ICP.icp(self.scan_h + 1.0, self.scan_h)
    self.gtsamAssertEquals(actual_transform, expected_transform, tol=1e-1)
  
  def test_icp_4(self):
    """
    Vertical line shifts 0.1 unit down.
    """
    scan_a = np.array([[0, 0, 0, 0],[0, 1, 2, 3], [1, 1, 1, 1]])
    scan_b = np.array([[0, 0, 0, 0],[-0.1, 0.9, 1.9, 2.9], [1, 1, 1, 1]])
    aTb = gtsam.Pose2(0, -0.1, 0)
    actual_transform = vanilla_ICP.icp(scan_a, scan_b)
    self.gtsamAssertEquals(actual_transform, aTb, tol=1e-2)
  
  def test_icp_5(self):
    """
    Horizontal line shifts 0.4 unit left.
    """
    scan_a = np.array([[0, 1, 2, 3],[0, 0, 0, 0], [1, 1, 1, 1]])
    scan_b = np.array([[-0.4, 0.6, 1.6, 2.6],[0, 0, 0, 0], [1, 1, 1, 1]])
    aTb = gtsam.Pose2(-0.4, 0, 0)
    actual_transform = vanilla_ICP.icp(scan_a, scan_b)
    self.gtsamAssertEquals(actual_transform, aTb, tol=1e-2)

if __name__ == "__main__":
  unittest.main()