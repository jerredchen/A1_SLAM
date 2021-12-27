"""
Feature detection of points in line segments in 2D LiDAR scans.
"""

from typing import List

import fractions
import numpy as np

class Line():
    """Class which represents a seed or line segment."""

    def __init__(self, scans, start_ind, end_ind):
        self.scans = scans
        self.start_ind = start_ind
        self.end_ind = end_ind
        self.coeffs = (0, 0, 0)
        self.fit(start_ind, end_ind)

    def fit(self, i, j):
        """Fits a set of line segment coefficients given a set of points."""
        if self.start_ind != i:
          self.start_ind = i
        if self.end_ind != j:
          self.end_ind = j
        m, b = np.polyfit(self.scans[0, i:j+1], self.scans[1, i:j+1], deg=1)
        frac = fractions.Fraction(m).limit_denominator(1000)
        A = -frac.numerator
        B = frac.denominator
        C = -b*frac.denominator
        self.coeffs = A, B, C
    
    def point_to_line(self, x, y):
        """Calculates the distance between a point and the line segment."""
        A, B, C = self.coeffs
        return abs(A*x+B*y+C)/np.sqrt(A**2+B**2)
    
    def endpoint_generation(self):
        """Generates endpoints of the line segment."""
        A, B, C = self.coeffs
        start_x = self.scans[0, self.start_ind]
        start_y = self.scans[1, self.start_ind]
        end_x = self.scans[0, self.end_ind]
        end_y = self.scans[1, self.end_ind]
        endpoint_x1 = ((B**2)*start_x - A*B*start_y - A*C) / (A**2 + B**2)
        endpoint_y1 = ((A**2)*start_y - A*B*start_x - B*C) / (A**2 + B**2)
        endpoint_x2 = ((B**2)*end_x - A*B*end_y - A*C) / (A**2 + B**2)
        endpoint_y2 = ((A**2)*end_y - A*B*end_x - B*C) / (A**2 + B**2)
        return np.array([[endpoint_x1, endpoint_x2], [endpoint_y1, endpoint_y2], [1, 1]])
    
    def points(self):
        """The 2D points associated with a line segment."""
        points = self.scans[:, self.start_ind:self.end_ind+1]
        return points
    
    def length(self):
        """The number of points within the line segment."""
        return self.end_ind - self.start_ind + 1
    
    def __eq__(self, other) -> bool:
        return self.start_ind == other.start_ind and self.end_ind == other.end_ind
    
    def __str__(self) -> str:
        first_str = f"Start ind/point: {self.start_ind}/{self.scans[:, self.start_ind]}, \
          End ind/point: {self.end_ind}\{self.scans[:, self.end_ind]}.\n"
        second_str = f"Ax+By+C=0, where A={self.coeffs[0]}, B={self.coeffs[1]}, C={self.coeffs[2]} \
          (y=mx+b where m={-self.coeffs[0]/self.coeffs[1]}, b={-self.coeffs[2]/self.coeffs[1]}).\n\n"
        return first_str + second_str

def seed_segment_detection(scans, angles, eps, delta, seed_points, min_points):
    """Determines whether a 'seed' segment can be found within a scan given strict requirements.

    Args:
        scans: The set of 2D points that is obtained after converting from LiDAR ranges.
        angles: The associated angles of every 2D point in the scan.
        eps: The threshold on the distance between a point Pk and its predicted point Pk'
              given the coefficients of the seed segment.
        delta: The threshold on the normal distance between a point Pk and coefficients of the seed segment.
        seed_points: The number of points to determine a seed segment.
        min_points: The minimum number of points necessary to determine a line segment.
    """
    seeds = []
    for i in range(0, len(scans[1])-min_points+1, 20):
        flag = True
        j = i + seed_points
        seed = Line(scans, i, j)
        A, B, C = seed.coeffs
        for k in range(i, j+1):
            # Obtain the predicted point Pk'
            theta = angles[k]
            point = scans[:,k]
            xk_prime = -C*np.cos(theta)/(A*np.cos(theta)+B*np.sin(theta))
            yk_prime = -C*np.sin(theta)/(A*np.cos(theta)+B*np.sin(theta))
            pointk_prime = np.array([xk_prime, yk_prime, 1])
            # d1 is the distance from Pk to Pk'
            d1 = np.linalg.norm(pointk_prime - point)
            if d1 > delta:
                flag = False
                break
            # d2 is distance from Pk to Seed(i,j)
            d2 = seed.point_to_line(point[0], point[1])
            if d2 > eps:
                flag = False
                break
        if flag:
            seeds.append(seed)
    return seeds

def region_growing(
    seed: Line,
    scans: np.ndarray,
    min_points: int,
    min_length: float,
    eps: float):
    """Grows each seed segment to be full line segment.

    Args:
        seed: The specific seed segment to be grown into a line segment.
        scans: The 2D points of a scan.
        min_points: The minimum number of points required to establish a line segment.
        min_length: The minimum length required to establish a line segment.
        eps: The threshold on the normal distance between a point and the line segment.
    """

    start = seed.start_ind + 1
    end = seed.end_ind - 1
    line = seed

    while end < len(scans[0]) and line.point_to_line(scans[0,end], scans[1, end]) < eps:
        line.fit(start, end)
        end += 1
    end -= 1
    while start > 0 and line.point_to_line(scans[0,start], scans[1, start]) < eps:
        line.fit(start, end)
        start -= 1
    start += 1
    num_points = end-start+1
    line_length = np.linalg.norm(scans[:,end] - scans[:,start])
    if num_points >= min_points and line_length >= min_length:
        return line

def remove_overlap(scans, lines: List[Line]):
  """Removes the overlapping regions between two consecutive line segments."""
  non_overlapping_lines = []
  for i in range(len(lines)-1):
      j = i + 1
      start_i = lines[i].start_ind
      end_i = lines[i].end_ind
      start_j = lines[j].start_ind
      end_j = lines[j].end_ind

      if start_j <= end_i:
          for k in range(start_j, end_i+1):
              dist_to_line_i = lines[i].point_to_line(scans[0,k], scans[1,k])
              dist_to_line_j = lines[j].point_to_line(scans[0,k], scans[1,k])
              if dist_to_line_i >= dist_to_line_j:
                  break
          end_i = k - 1
          start_j = k
      else:
          non_overlapping_lines.append(lines[i])
          non_overlapping_lines.append(lines[j])
          continue
      if start_i < end_i:
          lines[i].fit(start_i, end_i)
          non_overlapping_lines.append(lines[i])
      if start_j < end_j:
          lines[j].fit(start_j, end_j)
          non_overlapping_lines.append(lines[j])
  return non_overlapping_lines

def feature_detection(scans, angles):
    """Detecting points of defined line segments within a 2D LiDAR scan."""
    eps = 0.05
    delta = 0.25
    seed_points = 4
    min_points = 6
    min_length = 0.8
    seeds = seed_segment_detection(scans, angles, eps, delta, seed_points, min_points)
    lines = []
    for seed in seeds:
        new_line = region_growing(seed, scans, min_points, min_length, eps)
        if (new_line is not None) and ((len(lines) == 0) or (len(lines) > 0 and lines[-1] != new_line)):
            lines.append(new_line)
    updated_lines = remove_overlap(scans, lines)
    points = np.array([[],[],[]])
    for line in updated_lines:
        points = np.hstack((points, line.points()))
    return points
