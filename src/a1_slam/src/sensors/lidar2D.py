#!/usr/bin/env python

"""
Class for processing 2D LIDAR scans.
"""

from collections import deque
from threading import Lock

import gtsam
import numpy as np
import pygicp
import rospy
import tf2_ros
from gtsam.symbol_shorthand import X
from sensor_msgs import point_cloud2
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header

from optimization.optimizer import Optimizer


class Lidar2DWrapper:

    def __init__(self, optimizer: Optimizer):

        # Instantiate publisher attributes.
        self.scan_publisher = rospy.Publisher(
            "/transformed_clouds", PointCloud2, queue_size=5
        )

        # Instantiate trajectory and optimizer related attributes.
        self.state_index = 1
        self.optimizer = optimizer

        # Instantiate 2D LIDAR related attributes.
        self.submap_scans = []
        self.correspondence_threshold = 1.0
        self.icp_noise_model = gtsam.noiseModel.Diagonal.Sigmas(np.ones((6,)))
        self.baseTlaser = gtsam.Pose3()

        # Instantiate a threading lock.
        self.lidar2d_lock = Lock()

    def preprocess_measurement(self, laser_msg: LaserScan, min_range=0, max_range=np.inf):
        """Extracts ranges from the LaserScan message and converts into
        a 2*N homogenous matrix of points.

        Args:
            laser_msg: The LaserScan message from the A1.
        Returns:
            scan: A numpy array of shape (2, N), where N is the number of 2D points.
        """
        scan = np.zeros((2, len(laser_msg.ranges)), dtype=float)
        # Compute the 2D point where the angle starts at +90 deg, and
        # each range is spaced out in equal increments.
        for i, distance in enumerate(laser_msg.ranges):
            if min_range < distance < max_range:
                scan[0][i] = distance * \
                    np.cos(laser_msg.angle_min + i*laser_msg.angle_increment)
                scan[1][i] = distance * \
                    np.sin(laser_msg.angle_min + i*laser_msg.angle_increment)
        # Remove all values in the array that are still 0.
        mask = scan[0] != 0
        scan = scan[:, mask]

        return scan

    def create_lidar_factor(self,
                            a: int,
                            b: int,
                            scan_a: np.ndarray,
                            scan_b: np.ndarray,
                            aTb_estimate=None):
        """Creates an odometry factor from LIDAR measurements to be
        added to the factor graph.

        Args:
            a: The previous state index of the odometry factor.
            b: The subsequent state index of the odometry factor.
            scan_a: The LIDAR scan associated with state a.
            scan_b: The LIDAR scan associated with state b.
            aTb_estimate: Initial guess for ICP.
        Returns:
            factor: An odometry factor to be added to the factor graph.
            wTb_estimate: Initial estimate if a new state, None otherwise.
        """

        # Calculate the ICP initial estimate if not given.
        if not aTb_estimate:
            if self.optimizer.results.exists(X(b)):
                wTa = self.optimizer.results.atPose3(X(a))
                wTb = self.optimizer.results.atPose3(X(b))
                aTb_estimate = wTa.between(wTb)
            elif a == 0 and b == 1:
                aTb_estimate = gtsam.Pose3()
            else:
                wTp = self.optimizer.results.atPose3(X(a-1))
                wTq = self.optimizer.results.atPose3(X(b-1))
                aTb_estimate = wTp.between(wTq)

        # Use multithreaded GICP to calculate the rigid body transform.
        source = np.vstack((scan_b, np.ones(scan_b.shape[1])))
        target = np.vstack((scan_a, np.ones(scan_a.shape[1])))
        aTb_matrix = pygicp.align_points(
            target.T,
            source.T,
            max_correspondence_distance=self.correspondence_threshold,
            initial_guess=aTb_estimate.matrix(),
            k_correspondences=15,
            num_threads=4
        )

        # Set z-translation to be 0 and convert back into Pose3 object.
        aTb = gtsam.Pose3(aTb_matrix)
        rotation = aTb.rotation().Ypr(aTb.rotation().yaw(), 0, 0)
        aTb = gtsam.Pose3(rotation, np.array([aTb.x(), aTb.y(), 0.0]))

        factor = gtsam.BetweenFactorPose3(
            X(a), X(b), aTb, self.icp_noise_model)

        # Calculate the wTb pose estimate.
        wTa = self.optimizer.results.atPose3(X(a))
        wTb_estimate = wTa.compose(aTb)

        return factor, wTb_estimate

    def lidar_callback(self, msg: LaserScan, imu=None):
        """Processes the message from the LIDAR and create a IMU factor.

        Args:
            msg: a LaserScan ROS message.
            optimizer: an Optimizer object to add the factors and initial estimates.
            imu: if not None, an Imu object to use as an initial estimate for ICP.
        """

        # Calculate the aTb initial estimate from IMU if available.
        aTb_estimate = None
        index_a, index_b = self.state_index - 1, self.state_index
        if imu and len(self.submap_scans) > 0:
            aTb_estimate = imu.create_and_add_factor(index_a, index_b)

        # Obtain the minimum and maximum ranges to use preprocessing.
        min_range = rospy.get_param('/lidar2d/min_range')
        max_range = rospy.get_param('/lidar2d/max_range')

        # Preprocess the ranges, and transform the scan into the base_link frame.
        scan_b = self.preprocess_measurement(
            msg,
            min_range=min_range,
            max_range=max_range,
        )
        scan_b = self.baseTlaser.transformFrom(
            np.vstack((scan_b, np.ones(scan_b.shape[1]))))[:2]

        # Ignore if is the first received measurement.
        if len(self.submap_scans) == 0:
            with self.lidar2d_lock:
                self.submap_scans.append(scan_b)
            return
        scan_a = self.submap_scans[-1]

        # Create the LIDAR factor, add to the optimizer, and optimize.
        factor, wTb_estimate = self.create_lidar_factor(
            index_a,
            index_b,
            scan_a,
            scan_b,
            aTb_estimate
        )
        self.optimizer.add_factor(factor, (X(index_b), wTb_estimate))
        self.optimizer.optimize()
        with self.lidar2d_lock:
            self.submap_scans.append(scan_b)

        # Create skip connections to create a denser graph.
        with self.lidar2d_lock:
            current_state = self.state_index
            submap = self.submap_scans.copy()
        if len(submap) > 2:
            self.create_skip_connections(submap, current_state)
            self.optimizer.optimize()
        if len(submap) == rospy.get_param('/lidar2d/submap_length'):
            self.publish_transformed_scan(
                submap[0],
                current_state - len(submap) + 1
            )
        self.state_index += 1

    def create_skip_connections(self, submap, current_index):
        """
        Create skip connections between non-consecutive poses.

        Args:
            submap:        A list of the most recently recorded scans, 
                            where the last scan is the current scan.
            current_index: The state index associated with the current scan.
        """
        submap_length = len(submap)
        for i in range(submap_length - 2):
            index_a = current_index - i - 2
            index_b = current_index
            self.create_skip_connection(
                submap,
                index_a,
                index_b
            )

    def create_skip_connection(self, scans, index_a, index_b):
        """
        Create a particular skip connection between two poses.

        Args:
            scans:   A list of the most recently recorded scans,
                     where the last scan is the current scan.
            index_a: The index associated with a previous time step.
            index_b: The index associated with the current time step.
        """
        scan_a, scan_b = scans[-(index_b - index_a + 1)], scans[-1]
        factor, _ = self.create_lidar_factor(
            index_a,
            index_b,
            scan_a,
            scan_b
        )
        self.optimizer.add_factor(factor)

    def publish_transformed_scan(self, bTscan, index):
        """Transform a scan from the body frame into the world frame.

        Args:
            bTscan: The scan, currently in the body frame.
            index:  The state index of the to-be transformed scan.
        """

        # Obtain the optimized pose.
        wTb = self.optimizer.results.atPose3(X(index))

        # Transform the scan from the body frame to the world frame.
        wTscan = wTb.transformFrom(
            np.vstack((bTscan, np.zeros((1, bTscan.shape[1])))))

        # Reformat scan to be argument for creating point cloud.
        wTscan = wTscan.T
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        pcd = point_cloud2.create_cloud_xyz32(header, wTscan)
        self.scan_publisher.publish(pcd)

    # Instantiating parameters

    def initialize_params(self):
        """Initialize LIDAR parameters based on the set rosparams."""

        # Instantiate 2D LIDAR related attributes.
        self.submap_scans = deque(
            [], rospy.get_param('/lidar2d/submap_length'))

        # Parse the noise standard deviation parameters and create a GTSAM noise model.
        icp_noise_sigmas = rospy.get_param('/lidar2d/icp_noise')
        self.icp_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([
                1e-4,
                1e-4,
                np.deg2rad(icp_noise_sigmas[2]),
                icp_noise_sigmas[0],
                icp_noise_sigmas[1],
                1e-3
            ])
        )

        self.correspondence_threshold = rospy.get_param(
            '/lidar2d/correspondence_threshold')

        # Obtain the baseTlidar static transform and convert to a GTSAM Pose3.
        tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer)
        rospy.sleep(1) # Sleep for one second to allow static transform to publish.
        baseTlaser = tf_buffer.lookup_transform("base_link", "laser", rospy.Time())
        translation = np.array([
            baseTlaser.transform.translation.x,
            baseTlaser.transform.translation.y,
            baseTlaser.transform.translation.z,
        ])
        quaternion = np.array([
            baseTlaser.transform.rotation.w,
            baseTlaser.transform.rotation.x,
            baseTlaser.transform.rotation.y,
            baseTlaser.transform.rotation.z,
        ])
        self.baseTlaser = gtsam.Pose3(
            gtsam.Rot3.Quaternion(*quaternion),
            translation
        )

