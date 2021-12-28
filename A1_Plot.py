"""
Plotting utilities for A1 scripts.
"""

from typing import List

import matplotlib.pyplot as plt
import numpy as np

import gtsam
from gtsam.utils import plot

def plot_incremental_IMU_progress(values: gtsam.Values,
                                  start: int = 0,
                                  time_interval: float = 0.0) -> None:
    """Plots incremental trajectory of the robot with only an IMU."""

    fig = plt.figure(0)
    axes = fig.gca()

    poses = gtsam.utilities.allPose3s(values)
    keys = gtsam.KeyVector(poses.keys())

    # Plot the IMU trajectory.
    for key in keys[start:]:
        if values.exists(key):
            pose_i = values.atPose3(key)
            pose_prev = values.atPose3(key-1) if key > 0 else values.atPose3(0)
            axes.plot([pose_prev.x(), pose_i.x()],
                      [pose_prev.y(), pose_i.y()],
                      color='mediumseagreen')

    # Rescale the plot to include all points.
    axes.autoscale()

    # Pause in between each plot iteration.
    plt.pause(time_interval)
  
def plot_IMU_measurements(accel_measurements, gyro_measurements):
    """Plots the IMU measurements incrementally."""

    fig, axes = plt.subplots(2, 3)
    fig.set_tight_layout(True)

    accel_labels = ['x', 'y', 'z']
    accel_colors = ['r', 'b', 'g']
    gyro_colors = ['m', 'c', 'y']

    for i, label in enumerate(accel_labels):
        ax = axes[0][i]
        ax.clear()
        ax.set_xlabel(f'acceleration in {label}')
        for j, m in enumerate(accel_measurements):
            ax.scatter(j, m[i], color=accel_colors[i])

    for i, label in enumerate(accel_labels):
        ax = axes[1][i]
        ax.clear()
        ax.set_xlabel(f'ang vel about {label}')
        for j, m in enumerate(gyro_measurements):
            ax.scatter(j, m[i], color=gyro_colors[i])
  
def plot_final_IMU_traj(values: gtsam.Values) -> None:
    """Plots the final trajectory from using only the IMU."""
    fig = plt.figure(0)
    axes = fig.gca()

    poses = gtsam.utilities.allPose3s(values)
    keys = gtsam.KeyVector(poses.keys())

    # Plot the IMU trajectory.
    for key in keys:
        if values.exists(key):
            pose_i = values.atPose3(key)
            pose_prev = values.atPose3(key-1) if key > 0 else values.atPose3(0)
            axes.plot([pose_prev.x(), pose_i.x()],
                      [pose_prev.y(), pose_i.y()],
                      color='mediumseagreen')
    
    plt.show()

def plot_LIDAR_incremental_traj_and_map(values: gtsam.Values,
                                        scan: np.ndarray,
                                        start: int = 0,
                                        time_interval: float = 0.0) -> None:
    """Plot incremental progress of the robot from using LiDAR measurements."""
    fig = plt.figure(0)
    axes = fig.gca()

    poses = gtsam.utilities.allPose2s(values)
    keys = gtsam.KeyVector(poses.keys())

    # Plot the LiDAR scans to create a surrounding map.
    for i, key in enumerate(keys[start:]):
        # Transform the scan points from the local frame to the world frame.
        pose_i = values.atPose2(key)
        world_points = pose_i.matrix() @ scan
        axes.scatter(world_points[0], world_points[1], color='k', s=0.75)

    # Plot the trajectory in between the previous and current pose.
    for key in keys[start:]:
        if values.exists(key):
            pose_i = values.atPose2(key)
            pose_prev = values.atPose2(key-1) if key > 0 else values.atPose2(0)
            axes.plot([pose_prev.x(), pose_i.x()],
                      [pose_prev.y(), pose_i.y()],
                      color='mediumseagreen')

    # Rescale the plot to include all points.
    axes.autoscale()

    # Pause in between each plot iteration.
    plt.show()
    plt.pause(time_interval)

def plot_final_LIDAR_traj_and_map(values: gtsam.Values,
                                  measurements: List[np.ndarray]) -> None:
    """Plots the final trajectory from using LiDAR measurements."""
    fig = plt.figure()
    axes = fig.gca()

    poses = gtsam.utilities.allPose2s(values)
    keys = gtsam.KeyVector(poses.keys())

    # Plot the LiDAR scans to create a surrounding map.
    for i in range(len(keys)):
        if values.exists(keys[i]):
            # Transform the scan points from the local frame to the world frame.
            pose_i = values.atPose2(keys[i])
            points = measurements[i]
            world_points = np.apply_along_axis(pose_i.transformFrom, 1, points)
            axes.scatter(world_points[0], world_points[1], color='r', s=0.75)

    # Plot the trajectory in between the previous and current pose.
    for key in keys:
        if values.exists(key):
            pose_i = values.atPose2(key)
            pose_prev = values.atPose2(key-1) if key > 0 else values.atPose2(0)
            axes.plot([pose_prev.x(), pose_i.x()],
                      [pose_prev.y(), pose_i.y()],
                      color='mediumseagreen')

    plt.show()

def plot_dead_reckoning_map(pose,
                            scan,
                            scale: float = 0.1,
                            time_interval: float = 0.0) -> None:
    """Plots the map generated from LiDAR measurements using only dead reckoning."""
    fig = plt.figure(0)
    axes = fig.gca()

    # Plot the LiDAR scans to create a surrounding map.
    curr_world_points = pose.matrix() @ scan
    plt.scatter(curr_world_points[0], curr_world_points[1], color='r', s=0.75)

    # Plot the robot pose.
    plot.plot_pose2(0, pose, scale)

    # Rescale the plot to include all points.
    axes.autoscale()

    # Pause in between each plot iteration.
    plt.pause(time_interval)

def plot_full_incremental_traj_and_map(values: gtsam.Values,
                                       scan: np.ndarray,
                                       start: int = 0,
                                       time_interval: float = 0.0) -> None:
    """Plot the full SLAM incremental trajectory and map of the robot."""
    fig = plt.figure(0)
    axes = fig.gca()

    poses = gtsam.utilities.allPose3s(values)
    keys = gtsam.KeyVector(poses.keys())

    # Plot the LiDAR scans to create a surrounding map.
    for i, key in enumerate(keys[start:]):
        # Transform the scan points from the local frame to the world frame.
        pose_i = values.atPose3(key)
        world_points = pose_i.matrix() @ scan
        axes.scatter(world_points[0], world_points[1], color='k', s=0.75)

    # Plot the trajectory in between the previous and current pose.
    for key in keys[start:]:
        if values.exists(key):
            pose_i = values.atPose3(key)
            pose_prev = values.atPose3(key-1) if key > 0 else values.atPose3(0)
            axes.plot([pose_prev.x(), pose_i.x()],
                      [pose_prev.y(), pose_i.y()],
                      color='mediumseagreen')

    # Rescale the plot to include all points.
    axes.autoscale()

    # Pause in between each plot iteration.
    plt.show()
    plt.pause(time_interval)

def plot_final_full_traj_and_map(values: gtsam.Values,
                                 measurements: List[np.ndarray]) -> None:
    """Plot the full SLAM final trajectory and map of the robot."""
    fig = plt.figure()
    axes = fig.gca()

    poses = gtsam.utilities.allPose3s(values)
    keys = gtsam.KeyVector(poses.keys())

    # Plot the LiDAR scans to create a surrounding map.
    for i in range(len(keys)):
        if values.exists(keys[i]):
            # Transform the scan points from the local frame to the world frame.
            pose_i = values.atPose3(keys[i])
            world_points = pose_i.matrix() @ measurements[i]
            axes.scatter(world_points[0], world_points[1], color='r', s=0.75)

    # Plot the trajectory in between the previous and current pose.
    for key in keys:
        if values.exists(key):
            pose_i = values.atPose3(key)
            pose_prev = values.atPose3(key-1) if key > 0 else values.atPose3(0)
            axes.plot([pose_prev.x(), pose_i.x()],
                      [pose_prev.y(), pose_i.y()],
                      color='mediumseagreen')

    plt.show()