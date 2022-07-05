#!/usr/bin/env python

"""
Node for processing 2D LIDAR scans.
"""
import rospy

import gtsam
import numpy as np
from a1_slam.srv import AddFactor, GetResults
from collections import deque
from gtsam.symbol_shorthand import X
from registration import icp
from sensor_msgs import point_cloud2
from sensor_msgs.msg import LaserScan, PointCloud2
from sklearn.neighbors import NearestNeighbors
from std_msgs.msg import Header


class Lidar2DNode:

    def __init__(self):

        # Instantiate publisher and service attributes.
        self.scan_publisher = rospy.Publisher(
            "/transformed_clouds", PointCloud2, queue_size=5
        )
        self.request_optimizer = None
        self.request_results = None

        # Instantiate factor graph and optimizer attributes.
        self.state_index = 0
        self.results = gtsam.Values()

        # Instantiate 2D LIDAR related attributes.
        self.icp_noise_model = gtsam.noiseModel.Diagonal.Sigmas(np.ones((6,)))
        self.submap_scans = []

    def parse_config_parameters(self,
                                prior_pose_estimate,
                                prior_pose_sigmas,
                                icp_noise_sigmas):
        """Parse the config parameters to be used as gtsam objects.

        Args:
            prior_pose_estimate:    A list in format [x, y, theta]
                                        representing the initial estimate on the prior pose.
            prior_pose_sigmas:      A list in format [x, y, theta] representing the prior
                                        pose noise's translational and rotational standard deviation.
            icp_noise_sigmas:       A list in format [x, y, theta] representing the noise
                                        standard deviations associated with a LIDAR odometry factor.
        Returns:
            prior_pose_factor:      The prior factor associated with the initial pose.
            icp_noise_model:        A noise model representing the noise from LIDAR registration.
        """
        pose2d = gtsam.Pose2(
            prior_pose_estimate[0],
            prior_pose_estimate[1],
            np.deg2rad(prior_pose_estimate[2])
        )
        pose = gtsam.Pose3(pose2d)
        prior_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([
                np.deg2rad(prior_pose_sigmas[2]),
                1e-20,
                1e-20,
                prior_pose_sigmas[0],
                prior_pose_sigmas[1],
                1e-20
            ])
        )
        prior_pose_factor = gtsam.PriorFactorPose3(
            X(0), pose, prior_noise_model
        )
        icp_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([
                np.deg2rad(icp_noise_sigmas[2]),
                1e-20,
                1e-20,
                icp_noise_sigmas[0],
                icp_noise_sigmas[1],
                1e-20
            ])
        )
        return prior_pose_factor, icp_noise_model

    def preprocess_measurement(self, laser_msg: LaserScan):
        """Extracts ranges from the LaserScan message and converts into
        a 3*N homogenous matrix of points.
        Args:
            laser_msg: The LaserScan message from the A1.
        Returns:
            scan: A numpy array of shape (3, N), where N is the number of 2D points.
        """
        scan = np.zeros((2, len(laser_msg.ranges)), dtype=float)
        # Compute the 2D point where the angle starts at +90 deg, and
        # each range is spaced out in equal increments.
        min_range = rospy.get_param('/lidar2d/min_range')
        max_range = rospy.get_param('/lidar2d/max_range')
        for i, distance in enumerate(laser_msg.ranges):
            if min_range < distance < max_range:
                scan[0][i] = distance * \
                    np.cos(laser_msg.angle_min + i*laser_msg.angle_increment)
                scan[1][i] = distance * \
                    np.sin(laser_msg.angle_min + i*laser_msg.angle_increment)
        # Remove all values in the array that are still 0.
        mask = scan[0] != 0
        scan = scan[:, mask]

        # Perform radius-based outlier removal.
        knearest = rospy.get_param('/lidar2d/outlier_neighbors')
        radius = rospy.get_param('/lidar2d/outlier_radius')
        neighbors = NearestNeighbors(n_neighbors=knearest).fit(scan.T)
        inds = neighbors.radius_neighbors(radius=radius, return_distance=False)
        outlier_mask = [len(n) >= knearest for n in inds]
        scan = scan[:, outlier_mask]

        # Estimate the normal vectors if necessary.
        normals = None
        if rospy.get_param('/lidar2d/registration') == 'point-to-line':
            normal_kneighbors = rospy.get_param('/lidar2d/normal_kneighbors') 
            normals = icp.estimate_normals(scan, k_nearest=normal_kneighbors)
        return scan, normals

    def create_lidar_factor(self,
                            a: int,
                            b: int,
                            scan_a: np.ndarray,
                            scan_b: np.ndarray,
                            normals=None):
        """Creates an odometry factor from LIDAR measurements to be
        added to the factor graph.
        Args:
            a: The previous state index of the odometry factor.
            b: The subsequent state index of the odometry factor.
            scan_a: The LIDAR scan associated with state a.
            scan_b: The LIDAR scan associated with state b.
            registration: Specified ICP registration, default to point-to-line.
        Returns:
            factor: An odometry factor to be added to the factor graph.
            wTb_estimate: Initial estimate if a new state, None otherwise.
        """
        wTb_estimate = None
        if self.results.exists(X(b)):
            wTa, wTb = self.results.atPose3(X(a)), self.results.atPose3(X(b))
            init_aTb = wTa.between(wTb)
        elif b > 1:
            if X(b-1) != X(a):
                rospy.logerr(f"KEY A IS NOT AS EXPECTED")
            wTp, wTq = self.results.atPose3(
                X(b-2)), self.results.atPose3(X(b-1))
            init_aTb = wTp.between(wTq)
            wTb_estimate = wTq.compose(init_aTb)
        else:
            init_aTb = gtsam.Pose3()
            wTb_estimate = self.results.atPose3(X(0))
        # Convert into a Pose2 object for 2D ICP registration.
        init_aTb = gtsam.Pose2(
            init_aTb.x(),
            init_aTb.y(),
            init_aTb.rotation().yaw()
        )
        aTb_2D = icp.icp(scan_a, scan_b, init_aTb, normals)
        # Convert back into Pose3 object.
        aTb = gtsam.Pose3(aTb_2D)
        factor = gtsam.BetweenFactorPose3(
            X(a), X(b), aTb, self.icp_noise_model)
        return factor, wTb_estimate

    def lidar_callback(self, msg: LaserScan):
        """Processes the message from the LIDAR.
        1. Preprocesses ranges, turns into points.
        2. Create a corresponding factor and init estimate.
        3. Sends to optimizer server to update graph.
            - Receive updated results from optimizer.
        """
        scan, normals = self.preprocess_measurement(msg)
        if len(self.submap_scans) == 0:
            self.submap_scans.append((0, scan, normals))
            return
        previous_scan = self.submap_scans[-1][1]
        previous_normals = self.submap_scans[-1][2]
        factor, initial_estimate = self.create_lidar_factor(
            self.state_index - 1,
            self.state_index,
            previous_scan,
            scan,
            previous_normals
        )

        # Request optimized results from the optimizer service.
        serialized_factor = factor.serialize()
        serialized_estimate = initial_estimate.serialize()

        k1, k2 = self.state_index - 1, self.state_index
        rospy.loginfo(f"requesting results for {k1=}, {k2=}")
        response = self.request_optimizer(
            "BetweenFactorPose3",
            serialized_factor,
            serialized_estimate,
            X(self.state_index),
            -1,
            -1
        )
        received_results = gtsam.Values()
        received_results.deserialize(response.results)
        self.results = received_results
        self.submap_scans.append((self.state_index, scan, normals))
        self.state_index += 1
        rospy.loginfo(f"received results for {k1=}, {k2=}")
        self.optimize_submap_callback()

    def optimize_submap_callback(self, event=None):
        if len(self.submap_scans) <= 2:
            return
        submap = self.submap_scans.copy()
        curr_index, curr_scan, _ = submap[-1]
        for i in range(len(submap)-2):
            index, scan, normals = submap[i]
            factor, _ = self.create_lidar_factor(
                index,
                curr_index,
                scan,
                curr_scan,
                normals
            )

            # Request optimized results from the optimizer service.
            serialized_factor = factor.serialize()
            k1, k2 = index, curr_index
            rospy.loginfo(f"requesting results for {k1=}, {k2=}")
            try:
                response = self.request_optimizer(
                    "BetweenFactorPose3",
                    serialized_factor,
                    "",
                    -1,
                    -1,
                    -1
                )
                received_results = gtsam.Values()
                received_results.deserialize(response.results)
                self.results = received_results
                rospy.loginfo(f"received results for {k1=}, {k2=}")
            except rospy.service.ServiceException:
                rospy.logwarn(
                    "Service /optimizer_service returned no response")
        self.publish_transformed_scan(submap[0])

    def publish_transformed_scan(self, scan_pair):
        """Transform a scan into the world frame.
        Args:
            scan_pair: A tuple of (state_index, scan)
        """
        index, bTscan, _ = scan_pair
        # Obtain the pose of the body w.r.t. the world frame.
        wTb = self.results.atPose3(X(index))
        # Transform the scan from the body frame to the world frame.
        wTscan = wTb.transformFrom(np.vstack((bTscan, np.zeros((1, bTscan.shape[1])))))

        # Reformat scan to be argument for creating point cloud.
        wTscan = wTscan.T
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"
        pcd = point_cloud2.create_cloud_xyz32(header, wTscan)
        self.scan_publisher.publish(pcd)

    def launch_lidar_node(self):

        rospy.init_node('lidar_node', anonymous=True)

        self.submap_scans = deque(
            [], rospy.get_param('/lidar2d/submap_length'))

        # Instantiate service for optimizer.
        rospy.wait_for_service('optimizer_service')
        self.request_optimizer = rospy.ServiceProxy(
            'optimizer_service', AddFactor)
        rospy.wait_for_service('get_results_service')
        self.request_results = rospy.ServiceProxy(
            'get_results_service', GetResults)

        # Obtain the poses estimate, in meters and degrees.
        prior_pose_estimate = rospy.get_param('/prior_pose_estimate')

        # Obtain the pose's translational and rotational standard deviations, in meters and degrees.
        prior_pose_sigmas = rospy.get_param('/prior_pose_sigmas')

        # Obtain the standard deviation associated with registration noise, in meters and degrees.
        icp_noise_sigmas = rospy.get_param('/lidar2d/icp_noise')

        # Parse the config parameters to be GTSAM appropriate objects.
        prior_pose_factor, self.icp_noise_model = self.parse_config_parameters(
            prior_pose_estimate, prior_pose_sigmas, icp_noise_sigmas)

        # Request results with an added prior pose to the factor graph.
        response = self.request_results()
        received_results = gtsam.Values()
        received_results.deserialize(response.results)
        self.results = received_results
        self.state_index += 1

        topic = rospy.get_param('/lidar2d/topic')
        rospy.Subscriber(topic, LaserScan, self.lidar_callback)

        # rospy.Timer(rospy.Duration(0.1), self.optimize_submap_callback)

        rospy.spin()


if __name__ == "__main__":
    try:
        lidar_node = Lidar2DNode()
        lidar_node.launch_lidar_node()
    except rospy.ROSInterruptException:
        pass
