"""
Computing a resulting trajectory and map from the IRC (Intel Research Center) benchmark.
"""

import matplotlib.pyplot as plt
import numpy as np

import gtsam
import vanilla_ICP

def ranges_to_points(ranges) -> np.ndarray:
    """Convert the ranges from the LiDAR into 2D points in the local frame."""

    scan = np.zeros((2, 360), dtype=float)
    # Compute the 2D point where the angle starts at -90 deg, and
    # each range is spaced out in equal increments.
    for i, distance in enumerate(ranges):
        if 0.05 < distance < np.inf:
            scan[0][i] = distance*np.cos(-np.pi/2 + np.pi*i/180)
            scan[1][i] = distance*np.sin(-np.pi/2 + np.pi*i/180)
    # Remove all values in the array that are still 0.
    mask = scan[0] != 0
    scan = scan[:, mask]
    # Append ones to make the points homogenous.
    scan = np.vstack((scan, np.ones((1, len(scan[0])))))
    return scan

def IRC_traj_map_PersonalICP(start: int,
                              stop: int,
                              time_interval: float):
    """Plot the IRC trajectory and map with dead reckoning using the vanilla ICP implementation.

    Args:
        start: An integer representing the pose in the IRC dataset trajectory to start plotting.
        stop: An integer representing the pose in the IRC dataset trajectory to stop plotting.
        time_interval: The amount of seconds paused in between each plot iteration.
    """

    # Iterate through the .txt file and convert all ranges from the LiDAR into 2D points.
    measurements = []
    with open('data/intel_LASER_.txt') as f:
        for laser_line in f:
            ranges = [float(range.strip()) for range in laser_line.split()]
            measurements.append(ranges_to_points(ranges))

    plt.figure()
    plt.autoscale()

    pose = gtsam.Pose2()
    pose_prev = gtsam.Pose2()
    initial_transform = gtsam.Pose2()

    for i in range(start, stop+1):

        # Estimate the transform between two consecutive LiDAR measurements using Personal ICP.
        target_points = measurements[i]
        source_points = measurements[i+1]
        transform = vanilla_ICP.icp(source_points, target_points, initial_transform)

        # Compute the new pose through dead reckoning, and reinitialize for the next iteration.
        pose = pose.compose(transform)
        initial_transform = transform
        pose_prev = pose

        # Plot the trajectory from the previous pose to the current pose.
        x = [pose_prev.x(), pose.x()]
        y = [pose_prev.y(), pose.y()]
        plt.plot(x, y, c='mediumseagreen')

        # Transform the LiDAR scan points from the local frame to the global frame, and plot the resulting map.
        source_points = pose.matrix() @ source_points
        plt.scatter(source_points[0],source_points[1],marker=',',s=1.0,c='k')
        plt.pause(time_interval)
    plt.show()


if __name__ == "__main__":
    IRC_traj_map_PersonalICP(0, 355, 0.01)