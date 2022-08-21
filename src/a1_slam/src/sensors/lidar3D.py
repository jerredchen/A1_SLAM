#!/usr/bin/env python

"""
Class for processing 3D LIDAR clouds.
"""

from collections import deque
from threading import Lock

import gtsam
import numpy as np
import pygicp
import rospy
import tf2_ros
from gtsam.symbol_shorthand import X
from optimization.optimizer import Optimizer
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class Lidar3DWrapper:

    def __init__(self, optimizer: Optimizer):

        # Instantiate publisher attributes.
        self.cloud_publisher = rospy.Publisher(
            "/transformed_clouds", PointCloud2, queue_size=5
        )

        # Instantiate trajectory and optimizer related attributes.
        self.state_index = 1
        self.optimizer = optimizer

        # Instantiate 3D LIDAR related attributes.
        self.submap_clouds = []
        self.correspondence_threshold = 1.0
        self.icp_noise_model = gtsam.noiseModel.Diagonal.Sigmas(np.ones((6,)))
        self.baseTlidar = gtsam.Pose3()

        # Instantiate a threading lock.
        self.lidar3d_lock = Lock()

    def preprocess_measurement(self, laser_msg: PointCloud2, min_range=0, max_range=np.inf):
        """Extracts ranges from the Lasercloud message and converts into
        a 3*N matrix of points.

        Args:
            laser_msg: The PointCloud2 message from the A1.
        Returns:
            cloud: A numpy array of shape (3, N), where N is the number of 3D points.
        """
        point_vector_list = point_cloud2.read_points_list(laser_msg)
        points = [[point_vec.x, point_vec.y, point_vec.z]
                  for point_vec in point_vector_list
                  if min_range <= np.linalg.norm([
                      point_vec.x,
                      point_vec.y,
                      point_vec.z
                  ]) <= max_range]
        point_cloud = np.array(points).T

        return point_cloud

    def create_lidar_factor(self,
                            a: int,
                            b: int,
                            cloud_a: np.ndarray,
                            cloud_b: np.ndarray,
                            aTb_estimate=None):
        """Creates an odometry factor from LIDAR measurements to be
        added to the factor graph.

        Args:
            a: The previous state index of the odometry factor.
            b: The subsequent state index of the odometry factor.
            cloud_a: The LIDAR cloud associated with state a.
            cloud_b: The LIDAR cloud associated with state b.
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
        aTb_matrix = pygicp.align_points(
            cloud_a.T,
            cloud_b.T,
            max_correspondence_distance=self.correspondence_threshold,
            initial_guess=aTb_estimate.matrix(),
            k_correspondences=15,
            num_threads=2
        )

        # Convert back into Pose3 object.
        aTb = gtsam.Pose3(aTb_matrix)
        factor = gtsam.BetweenFactorPose3(
            X(a), X(b), aTb, self.icp_noise_model)

        # Calculate the wTb pose estimate.
        wTa = self.optimizer.results.atPose3(X(a))
        wTb_estimate = wTa.compose(aTb)

        return factor, wTb_estimate

    def lidar_callback(self, msg: PointCloud2, imu=None):
        """Processes the message from the LIDAR and create a IMU factor.

        Args:
            msg: a PointCloud2 ROS message.
            optimizer: an Optimizer object to add the factors and initial estimates.
            imu: if not None, an Imu object to use as an initial estimate for ICP.
        """

        # Calculate the aTb initial estimate from IMU if available.
        aTb_estimate = None
        index_a, index_b = self.state_index - 1, self.state_index
        if imu and len(self.submap_clouds) > 0:
            aTb_estimate = imu.create_and_add_factor(index_a, index_b)

        # Obtain the minimum and maximum ranges to use preprocessing.
        min_range = rospy.get_param('/lidar3d/min_range')
        max_range = rospy.get_param('/lidar3d/max_range')

        # Preprocess the ranges, and transform the cloud into the base_link frame.
        cloud_b = self.preprocess_measurement(
            msg,
            min_range=min_range,
            max_range=max_range,
        )
        cloud_b = self.baseTlaser.transformFrom(cloud_b)

        # Ignore if is the first received measurement.
        if len(self.submap_clouds) == 0:
            self.submap_clouds.append(cloud_b)
            return
        cloud_a = self.submap_clouds[-1]

        # Create the LIDAR factor, add to the optimizer, and optimize.
        factor, wTb_estimate = self.create_lidar_factor(
            index_a,
            index_b,
            cloud_a,
            cloud_b,
            aTb_estimate
        )
        self.optimizer.add_factor(factor, (X(index_b), wTb_estimate))
        self.optimizer.optimize()
        self.submap_clouds.append(cloud_b)

        # Create skip connections to create a denser graph.
        with self.lidar3d_lock:
            current_state = self.state_index
            submap = self.submap_clouds.copy()
        if len(submap) > 2:
            self.create_skip_connections(submap, current_state)
            self.optimizer.optimize()
        if len(submap) == rospy.get_param('/lidar3d/submap_length'):
            self.publish_transformed_cloud(
                submap[0],
                current_state - len(submap) + 1
            )
        self.state_index += 1

    def create_skip_connections(self, submap, current_index):
        """
        Create skip connections between non-consecutive poses.

        Args:
            submap:        A list of the most recently recorded clouds, 
                            where the last cloud is the current cloud.
            current_index: The state index associated with the current cloud.
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

    def create_skip_connection(self, clouds, index_a, index_b):
        """
        Create a particular skip connection between two poses.

        Args:
            clouds:   A list of the most recently recorded clouds,
                     where the last cloud is the current cloud.
            index_a: The index associated with a previous time step.
            index_b: The index associated with the current time step.
        """
        cloud_a, cloud_b = clouds[-(index_b - index_a + 1)], clouds[-1]
        factor, _ = self.create_lidar_factor(
            index_a,
            index_b,
            cloud_a,
            cloud_b
        )
        self.optimizer.add_factor(factor)

    def publish_transformed_cloud(self, bTcloud, index):
        """Transform a cloud from the body frame into the world frame.

        Args:
            bTcloud: The cloud, currently in the body frame.
            index:  The state index of the to-be transformed cloud.
        """

        # Obtain the optimized pose.
        wTb = self.optimizer.results.atPose3(X(index))

        # Transform the cloud from the body frame to the world frame.
        wTcloud = wTb.transformFrom(bTcloud)

        # Reformat cloud to be argument for creating point cloud.
        wTcloud = wTcloud.T
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        pcd = point_cloud2.create_cloud_xyz32(header, wTcloud)
        self.cloud_publisher.publish(pcd)

    # Instantiating parameters

    def initialize_params(self):
        """Initialize LIDAR parameters based on the set rosparams."""

        # Instantiate 3D LIDAR related attributes.
        self.submap_clouds = deque(
            [], rospy.get_param('/lidar3d/submap_length'))

        # Parse the noise standard deviation parameters and create a GTSAM noise model.
        icp_noise_sigmas = rospy.get_param('/lidar3d/icp_noise')
        self.icp_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([
                np.deg2rad(icp_noise_sigmas[0]),
                np.deg2rad(icp_noise_sigmas[1]),
                np.deg2rad(icp_noise_sigmas[2]),
                icp_noise_sigmas[0],
                icp_noise_sigmas[1],
                icp_noise_sigmas[2]
            ])
        )

        self.correspondence_threshold = rospy.get_param(
            '/lidar3d/correspondence_threshold')

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
