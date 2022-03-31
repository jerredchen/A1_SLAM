"""
Incrementally parsing ROSbag to obtain 2D LiDAR scans and produce factor graph solution
using relative pose transforms obtained from the LiDAR measurements.
"""

import matplotlib.pyplot as plt
import numpy as np
import rosbag

import A1_Plot
import gtsam
import vanilla_ICP

def plot_ICP_correspondences(bag_name, topic_name, i):
    """Plot the correspondences between two consecutive measurements."""
    bag = rosbag.Bag(bag_name)
    messages = list(bag.read_messages(topic_name))
    target_ranges = messages[i][1].ranges
    target_scan = ranges_to_points(target_ranges)
    source_ranges = messages[i+1][1].ranges
    source_scan = ranges_to_points(source_ranges)
    initial_transform = gtsam.Pose2()
    vanilla_ICP.icp(source_scan,
                     target_scan,
                     initial_transform=initial_transform,
                     max_iterations=25,
                     plotPoints=True)
    plt.show()
    bag.close()

def ranges_to_points(ranges) -> np.ndarray:
    """Convert the ranges from the LiDAR into 2D points in the local frame."""
    scan = np.zeros((2, 360), dtype=float)
    # Compute the 2D point where the angle starts at +90 deg, and
    # each range is spaced out in equal increments.
    for i, distance in enumerate(ranges):
        if 0.05 < distance < np.inf:
            scan[0][i] = distance*np.cos(np.pi - np.pi*i/180)
            scan[1][i] = distance*np.sin(np.pi - np.pi*i/180)
    # Remove all values in the array that are still 0.
    mask = scan[0] != 0
    scan = scan[:, mask]
    # Append ones to make points homogenous.
    scan = np.vstack((scan, np.ones((1, len(scan[0])))))
    return scan

def optimize_trajectory(bag_name: str,
                        topic_name: str,
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
    bag = rosbag.Bag(bag_name)

    # Instantiate the factor graph and its initial estimates container.
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()

    # Instantiate the iSAM2 parameters to create the iSAM2 object.
    parameters = gtsam.ISAM2Params()
    parameters.setRelinearizeThreshold(0.1)
    # parameters.setRelinearizeSkip(1)
    isam = gtsam.ISAM2(parameters)

    # Declare noise models for the prior pose and the associated noise with ICP.
    PRIOR_POSE_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([prior_x_pose_sigma, prior_y_pose_sigma, prior_heading_sigma * np.pi / 180]))
    ICP_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-5, 1e-5, 1e-6]))

    # Add the prior factor to the factor graph with its associated initial estimate.
    graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(), PRIOR_POSE_NOISE))
    initial_estimate.insert(0, gtsam.Pose2())
    isam.update(graph, initial_estimate)
    initial_estimate.clear()
    result = isam.calculateEstimate()

    # Initialize the transforms needed before performing any optimization.
    scan_transform = gtsam.Pose2()
    skip_transform = gtsam.Pose2()

    for k, bag_info in enumerate(bag.read_messages(topic_name)):

        # Extract the LiDAR message from the bag at a particular time instance.
        msg = bag_info[1]

        # Initialize scans from the previous iteration.
        if k > 1:
            scan_pprev = scan_prev
        if k > 0:
            scan_prev = scan

        # Convert the scan from a 1D array of ranges to an array of 2D points in the local pose frame.
        scan = ranges_to_points(msg.ranges)

        if k > 0:
            # Estimate the transform between two consecutive scan measurements.
            scan_transform = vanilla_ICP.icp(scan, scan_prev, initial_transform=scan_transform)

            # Add an odometry factor between two poses and its initial estimate from dead reckoning.
            graph.add(gtsam.BetweenFactorPose2(k - 1, k, scan_transform, ICP_NOISE))
            initialized_odom = result.atPose2(k - 1).compose(scan_transform)
            initial_estimate.insert(k, initialized_odom)
            if k > 1:
                skip_transform = vanilla_ICP.icp(scan, scan_pprev, initial_transform=skip_transform)
                graph.add(gtsam.BetweenFactorPose2(k - 2, k, skip_transform, ICP_NOISE))

            # Perform an iSAM2 incremental update.
            isam.update(graph, initial_estimate)
            result = isam.calculateEstimate()

            # Plot the resulting trajectory and map from the new updated estimates.
            A1_Plot.plot_LIDAR_incremental_traj_and_map(result, scan, k, 0.1)

            # Clear the graph and initial estimates.
            graph = gtsam.NonlinearFactorGraph()
            initial_estimate.clear()

    bag.close()

def dead_reckoning(bag_name: str,
                   topic_name: str):
    """Plot the A1 trajectory and map without optimization (only dead reckoning)."""

    # Open the ROS bag to be used.
    bag = rosbag.Bag(bag_name)
    scan_transform = gtsam.Pose2()
    pose = gtsam.Pose2()

    for k, bag_info in enumerate(bag.read_messages(topic_name)):

        # Extract the LiDAR message from the bag at a particular time instance.
        msg = bag_info[1]

        # Initialize scans from the previous iteration.
        if k > 0:
            scan_prev = scan

        # Convert the scan from a 1D array of ranges to an array of 2D points in the local pose frame.
        scan = ranges_to_points(msg.ranges)

        if k > 0:
            # Estimate the transform between two consecutive scan measurements.
            scan_transform = vanilla_ICP.icp(scan, scan_prev, scan_transform)
            # Compute the next pose through dead reckoning, and plot the resulting map.
            pose = pose.compose(scan_transform)
            A1_Plot.plot_dead_reckoning_map(pose, scan, 0.1, 0.01)
    plt.show()

    bag.close()

if __name__ == "__main__":
    plot_ICP_correspondences('data/bags/A1_bag_square_traj_2020-09-04-18-03-08.bag',
                                '/slamware_ros_sdk_server_node/scan', 51)
    # optimize_trajectory('data/bags/A1_bag_square_traj_2020-09-04-18-03-08.bag',
    #                     '/slamware_ros_sdk_server_node/scan')
