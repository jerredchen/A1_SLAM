"""
Incrementally parsing ROSbag to obtain 2D LiDAR scans and produce factor graph solution
using relative pose transforms obtained from the LiDAR measurements.
"""

from collections import deque

import gtsam
import matplotlib.pyplot as plt
import numpy as np
import rosbag
import sgdicp2D
from sklearn.neighbors import NearestNeighbors
from src.a1_pose_slam.src.registration import icp_line, vanilla_ICP
from src.a1_pose_slam.src.utils import A1_Plot

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
        if 1.5 < distance < 5:
            scan[0][i] = distance*np.cos(np.pi - np.pi*i/180)
            scan[1][i] = distance*np.sin(np.pi - np.pi*i/180)
    # Remove all values in the array that are still 0.
    mask = scan[0] != 0
    scan = scan[:, mask]
    # Append ones to make points homogenous.
    scan = np.vstack((scan, np.ones((1, len(scan[0])))))
    return scan

def remove_outliers(scan, radius, num_points) -> np.ndarray:
    """Remove outliers from the scan."""
    scan_T = scan[:].T
    neighbors = NearestNeighbors(n_neighbors=num_points + 1, radius=1.0).fit(scan_T)

    dists, _ = neighbors.kneighbors(scan.T, return_distance=True)
    mask = (dists <= radius).all(axis=1)
    return scan[:, mask]

def optimize_trajectory(bag_name: str,
                        topic_name: str,
                        skip_steps: int,
                        n: int):
    """Incrementally optimize the A1's trajectory from LiDAR measurements.

    Args:
        bag_name: The name of the ROS bag that was collected over the course of a trajectory.
        topic_name: The name of the LiDAR topic which consists of the recorded LiDAR scans.
        skip_steps: The maximum number of poses away from the current pose to form skip connections.
        n: Start index if you do not want to start at the beginning of the bag.
    """

    # Open the ROS bag to be used.
    bag = rosbag.Bag(bag_name)

    # Instantiate the factor graph and its initial estimates container.
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()

    # Instantiate the iSAM2 parameters to create the iSAM2 object.
    parameters = gtsam.ISAM2Params()
    parameters.setRelinearizeThreshold(0.1)
    isam = gtsam.ISAM2(parameters)

    # Declare noise models for the prior pose and the associated noise with ICP.
    PRIOR_XY_SIGMA = 1e-8
    PRIOR_THETA_SIGMA = 1e-9
    PRIOR_POSE_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([PRIOR_XY_SIGMA, PRIOR_XY_SIGMA, np.deg2rad(PRIOR_THETA_SIGMA)]))

    # Add the prior factor to the factor graph with its associated initial estimate.
    graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(), PRIOR_POSE_NOISE))
    initial_estimate.insert(0, gtsam.Pose2())
    isam.update(graph, initial_estimate)
    initial_estimate.clear()
    result = isam.calculateEstimate()

    # Initialize the transform needed before performing any optimization.
    prev_transform = gtsam.Pose2()
    stored_scans= deque([], maxlen=skip_steps)
    stored_timestamps = deque([], maxlen=skip_steps)
    all_scans = []
    dist_threshold = 0.10
    rot_threshold = np.pi/12
    # plt.figure()
    # plt.autoscale()

    for k, bag_info in enumerate(bag.read_messages(topic_name)):

        # Extract the LiDAR message from the bag at a particular time instance.
        msg = bag_info[1]

        # Convert the scan from a 1D array of ranges to an array of 2D points in the local pose frame.
        source_scan = ranges_to_points(msg.ranges)
        # print(f"Shape of unfiltered scan: {source_scan.shape}")
        # source_scan = remove_outliers(source_scan, 0.70, 2)
        # print(f"Shape of filtered scan: {source_scan.shape}")

        if k == 0:
            stored_scans.append(source_scan)
        
        if k <= n:
            stored_scans.append(source_scan)
            continue

        for i in range(min(k - n, skip_steps)):
            # Estimate the transform between two consecutive scan measurements.
            # scan_transform = vanilla_ICP.icp(scan, scan_prev, initial_transform=scan_transform)
            # scan_transform = icp_line.icp(scan, scan_prev, initial_transform=scan_transform)
            print(f"********************{k - n} and {k - n - (i + 1)}********************")
            # >>>>>>> Remove for Bayesian ICP
            # target_scan = stored_scans[-(i + 1)]
            if i == 0:
                init_estimate = [prev_transform.x(), prev_transform.y(), prev_transform.theta()]
                # targetTsource = icp_line.icp(source_scan, target_scan, prev_transform)
            else:
                wTb = result.atPose2(k - n)
                wTa = result.atPose2(k - n - (i + 1))
                aTb_mat = wTa.inverse().matrix() @ wTb.matrix()
                init_estimate = [aTb_mat[0,2], aTb_mat[1,2], np.arctan2(aTb_mat[1,0], aTb_mat[0,0])]
                # init_estimate = gtsam.Pose2(np.arctan2(aTb_mat[1,0], aTb_mat[0,0]), aTb_mat[:2,2])
                # targetTsource = icp_line.icp(source_scan, target_scan, init_estimate)
            target_scan = stored_scans[-(i + 1)]
            for _ in range(3):
                samples = sgdicp2D.bayesian_icp(source_scan, target_scan, init_estimate, [0.0, 0.0, 0.0], [100, 100, 100])
                if not (samples[0] == -1.0).all():
                    break
            if (samples[0] == -1.0).all() and i == 0:
                targetTsource = icp_line.icp(source_scan, target_scan, initial_transform=prev_transform)
                cov = np.eye(3) * 0.1
            elif (samples[0] == -1.0).all() and i > 0:
                continue
            else:
                mean_params = np.mean(samples[200:,:], axis=0)
                cov = np.cov(samples.T)

                std_devs = np.sqrt(np.diag(cov))

                targetTsource = gtsam.Pose2(mean_params[0], mean_params[1], mean_params[2])
                print(f"computed transform: {targetTsource}")

                # plt.figure()
                # transformed_scan = targetTsource.matrix() @ source_scan
                # plt.scatter(source_scan[0], source_scan[1])
                # plt.scatter(target_scan[0], target_scan[1])
                # plt.scatter(transformed_scan[0], transformed_scan[1])
                # plt.legend(["Source scan", "Target scan", "Bayesian transformed result"])
                # plt.savefig(f'results/scans_{k}_{k - (i + 1)}.png')
                # plt.close()

            # Add an odometry factor between two poses.
            icp_noise = gtsam.noiseModel.Gaussian.Covariance(cov)
            graph.add(gtsam.BetweenFactorPose2(k - n - (i + 1), k - n, targetTsource, icp_noise))
            if i == 0:                    
                initialized_odom = result.atPose2(k - n - 1).compose(targetTsource)
                initial_estimate.insert(k - n, initialized_odom)
                prev_transform = targetTsource

            # Perform an iSAM2 incremental update.
            isam.update(graph, initial_estimate)
            result = isam.calculateEstimate()

            # Clear the graph and initial estimates.
            graph = gtsam.NonlinearFactorGraph()
            initial_estimate.clear()

            # Plot the resulting trajectory and map from the new updated estimates.
            A1_Plot.plot_LIDAR_incremental_traj_and_map(result, source_scan, k - n, 0.01)
            
            # Append scan
            stored_scans.append(source_scan)

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

        if k < 60:
            continue

        if k > 0:
            # Estimate the transform between two consecutive scan measurements.
            # scan_transform = vanilla_ICP.icp(scan, scan_prev, scan_transform)
            init_estimate = [scan_transform.x(), scan_transform.y(), scan_transform.theta()]
            samples = sgdicp2D.bayesian_icp(scan, scan_prev, init_estimate, [0.0, 0.0, 0.0], [100, 100, 100])
            if len(samples) < 100:
                pass
            else:
                mean_params = np.mean(samples[100:,:], axis=0)
                scan_transform = gtsam.Pose2(mean_params[0], mean_params[1], mean_params[2])
            # Compute the next pose through dead reckoning, and plot the resulting map.
            pose = pose.compose(scan_transform)
            A1_Plot.plot_dead_reckoning_map(pose, scan, 0.1, 0.01)
    plt.show()

    bag.close()

if __name__ == "__main__":
    optimize_trajectory('../../bags/A1_walk_dataset_04_15_22/A1_bag_fast1_2022-03-05-07-01-08.bag',
                        '/slamware_ros_sdk_server_node/scan', 6, 40)
    # dead_reckoning('../../bags/A1_walk_dataset_04_15_22/A1_bag_slow1_2022-03-05-06-54-59.bag',
    #                     '/slamware_ros_sdk_server_node/scan')
