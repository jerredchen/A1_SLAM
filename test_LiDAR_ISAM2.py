import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase

class TestLiDAR(GtsamTestCase):

  def setUp(self):
    self.path_a = [
      gtsam.Pose2(),
      gtsam.Pose2(1, 0, 0),
      gtsam.Pose2(1, 0, np.pi/2),
      gtsam.Pose2(1, 1, np.pi/2),
      gtsam.Pose2(1, 1, np.pi),
      gtsam.Pose2(0, 1, np.pi)
    ]
    # Measurements are translated one unit left and one unit right, represented in body frame.
    self.points_a = [np.array([[0,0],[1,-1]]) for _ in range(len(self.path_a))]
  
  def test_a(self):
    ground_truth = [
      np.array([[0,0],[1,-1]]),
      np.array([[1,1],[1,-1]]),
      np.array([[0,2],[0,0]]),
      np.array([[0,2],[1,1]]),
      np.array([[1,1],[0,2]]),
      np.array([[0,0],[0,2]])
    ]
    world_points = []
    for i, pose in enumerate(self.path_a):
      points = self.points_a[i]
      points_homog = np.vstack((points, np.ones(points.shape[1])))
      world_points.append((pose.matrix() @ points_homog)[:2])
    np.testing.assert_allclose(world_points, ground_truth, atol=1e-07)

if __name__ == "__main__":
  unittest.main()