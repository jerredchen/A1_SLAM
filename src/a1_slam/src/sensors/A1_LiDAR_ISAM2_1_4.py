"""
Incrementally parsing ROSbag to obtain 2D LiDAR scans and produce factor graph solution
using relative pose transforms obtained from the LiDAR measurements.
"""
from collections import deque
import matplotlib.pyplot as plt
import numpy as np
import time
import pandas as pd

import gtsam
import vanilla_ICP
import icp_line
import A1_Plot
import rosbag

def ranges_to_points(ranges) -> np.ndarray:
    """Convert the ranges from the LiDAR into 2D points in the local frame."""
    scan = np.zeros((2, 360), dtype=float)
    # Compute the 2D point where the angle starts at +90 deg, and
    # each range is spaced out in equal increments.
    for i, distance in enumerate(ranges):
        #if 0.05 < distance < np.inf:
        if 1.5 < distance < 5:
            scan[0][i] = distance*np.cos(np.pi - np.pi*i/180)
            scan[1][i] = distance*np.sin(np.pi - np.pi*i/180)
    # Remove all values in the array that are still 0.
    mask = scan[0] != 0
    scan = scan[:, mask]
    # Append ones to make points homogenous.
    scan = np.vstack((scan, np.ones((1, len(scan[0])))))
    return scan

def optimize_trajectory(bag_name: str,
                        prior_x_pose_sigma = 1e-5,
                        prior_y_pose_sigma = 1e-5,
                        prior_heading_sigma = 1e-4):
    """Incrementally optimize the A1's trajectory from LiDAR measurements.

    Args:
        bag_name: The name of the ROS bag that was collected over the course of a trajectory.
        topic_name: The name of the LiDAR topic which consists of the recorded LiDAR scans.
        prior_x_pose_sigma: The standard deviation of the noise model for the prior pose's x-direction, in m.
        prior_y_pose_sigma: The standard deviation of the noise model for the prior pose's y-direction, in m.
        prior_heading_sigma: The standard deviation of the noise model for the prior pose's heading, in deg.
    """

    # Open the ROS bag to be used.
    bag = rosbag.Bag('/home/jerredchen/borglab/A1_SLAM/bags/A1_walk_dataset_04_15_22/A1_bag_slow2_2022-03-05-07-02-26.bag')

    # print("Total Scans::", len(measurements))
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
    ICP_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-1, 1e-1, 1e-1]))
    #ICP_NOISE = gtsam.noiseModel.Gaussian.Covariance(np.matrix([[1e-6, 1e-6, 1e-6], [1e-6, 1e-6, 1e-6], [1e-6, 1e-6, 1e-6]]))
    #ICP_noise = gtsam.noiseModel.Robust.unwhiten(np.array([1e-10, 1e-10, 1e-10]))

    # Add the prior factor to the factor graph with its associated initial estimate.
    graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(), PRIOR_POSE_NOISE))
    initial_estimate.insert(0, gtsam.Pose2())
    isam.update(graph, initial_estimate)
    initial_estimate.clear()
    result = isam.calculateEstimate()

    # Initialize the transforms needed before performing any optimization.
    aTb = gtsam.Pose2()
    stored_scans= deque([], maxlen=6)
    all_scans = []
    scan_prev = gtsam.Pose2()
    timestamps = []

    # for k, bag_info in enumerate(measurements):
    for k, bag_info in enumerate(bag.read_messages('/slamware_ros_sdk_server_node/scan')):

        if k >= 200:
            break

        # Extract the LiDAR message from the bag at a particular time instance.
        msg = bag_info[1]

        # Convert the scan from a 1D array of ranges to an array of 2D points in the local pose frame.
        scan_b = ranges_to_points(msg.ranges)
        timestamps.append(msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9)

        if k == 0:
            stored_scans.append(scan_b)
            all_scans.append(scan_b)
            continue

        for i in range(min(k, 6)):
            print(f"*******************scans {k}, {k-(i+1)}*******************")
            if i == 0:
                init_estimate = scan_prev
            else:
                wTb = result.atPose2(k)
                wTa = result.atPose2(k - (i + 1))
                init_estimate = wTa.between(wTb)
            # Estimate the transform between two consecutive scan measurements.
            # scan_transform, _, skip = icp_line.icp(scan, stored_scans[-(i + 1)], initial_transform=init_estimate)
            # scan_a = measurements[k - (i + 1)]
            scan_a = stored_scans[-(i + 1)]
            # aTb = vanilla_ICP.icp(scan_b, scan_a, initial_transform=init_estimate)
            aTb = icp_line.icp(scan_b, scan_a, initial_transform=init_estimate)
            # Add an odometry factor between two poses and its initial estimate from dead reckoning.
            graph.add(gtsam.BetweenFactorPose2(k - (i + 1), k, aTb, ICP_NOISE))
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

            # Plot the resulting trajectory and map from the new updated estimates.
            # A1_Plot.plot_LIDAR_incremental_traj_and_map(result, scan_b, k, 0.1)

        stored_scans.append(scan_b)
        all_scans.append(scan_b)

    A1_Plot.plot_final_LIDAR_traj_and_map(result, all_scans)

def dead_reckoning(scan_name: str):
    """Plot the A1 trajectory and map without optimization (only dead reckoning)."""

    measurements = []
    with open(scan_name) as f:
      for laser_line in f:
        ranges = [float(range.strip()) for range in laser_line.split()]
        measurements.append(ranges_to_points(ranges))
    print("Number of scans:", len(measurements))
    scan_transform = gtsam.Pose2()
    pose = gtsam.Pose2()

    k = 0

    for k in range(len(measurements)):
        # Initialize scans from the previous iteration.
        if k > 0:
            scan_prev = scan

        # Convert the scan from a 1D array of ranges to an array of 2D points in the local pose frame.
        scan = measurements[k]

        if k > 0:
            # Estimate the transform between two consecutive scan measurements.
            #scan_transform = Personal_ICP.icp(scan, scan_prev, scan_transform)
            scan_transform, scan_c = icp_line.icp(scan, scan_prev, scan_transform)
            # Compute the next pose through dead reckoning, and plot the resulting map.
            pose = pose.compose(scan_transform)
            A1_Plot.plot_dead_reckoning_map(pose, scan_c, 0.1, 0.01)
            print(k, scan_transform)
        #if(k%10==0): print("Processing Scan:", k)
    print("Done")
    plt.show()


if __name__ == "__main__":
    #dead_reckoning('../data/bags/A1_walkingDataset/A1_bag_slow_2022-03-05-06-53-41/slamware_ros_sdk_server_node-scan.txt')
    #dead_reckoning('../data/bags/A1_walkingDataset/A1_bag_fast1_2022-03-05-07-01-08/slamware_ros_sdk_server_node-scan.txt')
    #dead_reckoning('data/bags/A1_walkingDataset/A1_walk1_2022-03-05-07-20-21/slamware_ros_sdk_server_node-scan.txt')
    #dead_reckoning('data/bags/A1_walkingDataset/A1_bag_slow_2022-03-05-06-53-41/slamware_ros_sdk_server_node-scan.txt')
    #optimize_trajectory('../data/bags/A1_walkingDataset/A1_bag_fast1_2022-03-05-07-01-08/slamware_ros_sdk_server_node-scan.txt',
    #                     '/slamware_ros_sdk_server_node/scan')
    # optimize_trajectory('../data/bags/A1_walkingDataset/A1_bag_fast1_2022-03-05-07-01-08/slamware_ros_sdk_server_node-scan.txt')
    optimize_trajectory('scans.txt')
    #optimize_trajectory('../data/bags/A1_walkingDataset/A1_bag_medium1_2022-03-05-06-58-18/slamware_ros_sdk_server_node-scan.txt')
    #optimize_trajectory('../data/bags/A1_walkingDataset/A1_bag_slow_2022-03-05-06-53-41/slamware_ros_sdk_server_node-scan.txt')
    #optimize_trajectory('data/bags/A1_walkingDataset/A1_walk1_2022-03-05-07-20-21/slamware_ros_sdk_server_node-scan.txt')
