from collections import deque
from threading import Lock

import gtsam
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from gtsam.symbol_shorthand import B, V, X
from nav_msgs.msg import Path


class Optimizer:
    def __init__(self) -> None:
        # Instantiate relevant optimizer data structures.
        self.factor_queue = deque([])
        self.poses_queue = deque([])

        # Instantiate factor graph data structures.
        self.isam = gtsam.ISAM2(gtsam.ISAM2Params())
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()
        self.results = gtsam.Values()

        # Instantiate publishers.
        self.pose_publisher = rospy.Publisher(
            "/pose_estimate", PoseStamped, queue_size=5
        )
        self.pose_cov_publisher = rospy.Publisher(
            "/pose_cov_estimate", PoseWithCovarianceStamped, queue_size=5
        )
        self.traj_publisher = rospy.Publisher(
            "/traj_estimate", Path, queue_size=5
        )

        # Instantiate a threading lock.
        self.optimizer_lock = Lock()

    ###################### Optimization Functions ######################

    def add_prior_factors(self):
        """Parse the parameters and add prior factors to the graph."""

        # Parse the prior pose estimate and standard deviations parameters.
        prior_pose_estimate = rospy.get_param('/prior_pose_estimate')
        prior_pose_sigmas = rospy.get_param('/prior_pose_sigmas')
        rpy = np.deg2rad(prior_pose_estimate[3:])
        pose_rotation = gtsam.Rot3.Ypr(*(rpy[::-1]))
        pose_translation = gtsam.Point3(*prior_pose_estimate[:3])
        prior_pose_estimate = gtsam.Pose3(pose_rotation, pose_translation)
        prior_pose_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.concatenate(
                (np.deg2rad(prior_pose_sigmas[3:]), prior_pose_sigmas[:3]))
        )

        # Create a GTSAM PriorFactorPose3 and add the factor to the factor graph.
        prior_pose_factor = gtsam.PriorFactorPose3(
            X(0), prior_pose_estimate, prior_pose_noise)
        self.add_factor(prior_pose_factor, (X(0), prior_pose_factor.prior()))

        # Parse and add velocity and bias priors if using an IMU for SLAM.
        if rospy.get_param('/use_imu'):
            # Parse the velocity parameters and create a GTSAM velocity factor.
            prior_vel_estimate = rospy.get_param('/imu/prior_vel_estimate')
            prior_vel_sigmas = rospy.get_param('/imu/prior_vel_sigmas')
            prior_vel_estimate = np.array(prior_vel_estimate)
            prior_vel_noise = gtsam.noiseModel.Diagonal.Sigmas(
                np.array(prior_vel_sigmas))
            prior_vel_factor = gtsam.PriorFactorVector(
                V(0), prior_vel_estimate, prior_vel_noise)

            # Parse the bias parameters and create a GTSAM bias factor.
            prior_bias_estimate = rospy.get_param('/imu/prior_bias_estimate')
            prior_bias_sigmas = rospy.get_param('/imu/prior_bias_sigmas')
            accel_bias, gyro_bias = prior_bias_estimate[:3], prior_bias_estimate[3:]
            prior_bias_estimate = gtsam.imuBias.ConstantBias(
                accel_bias, gyro_bias)
            prior_bias_noise = gtsam.noiseModel.Diagonal.Sigmas(
                np.array(prior_bias_sigmas))
            prior_bias_factor = gtsam.PriorFactorConstantBias(
                B(0), prior_bias_estimate, prior_bias_noise)

            # Add the priors to the factor graph.
            self.add_factor(prior_vel_factor, (V(0), prior_vel_factor.prior()))
            self.add_factor(prior_bias_factor, (B(0), prior_bias_factor.prior()))
        self.optimize()

    def add_factor(self, factor, init_estimate_pairs=None):
        """Add a factor and initial estimates to the optimizer queue.

        Args:
            factor: A gtsam factor to be added to the factor graph.
            init_estimate_pairs: Optional tuple or a list of tuples 
                                 in the format of (key, initial estimate). 
        """
        if init_estimate_pairs is None:
            init_estimate_pairs = []
        elif type(init_estimate_pairs) != list:
            init_estimate_pairs = [init_estimate_pairs]
        self.factor_queue.append((factor, init_estimate_pairs))

    def optimize(self):
        """Optimized queued factors and estimates using iSAM2."""
        # Add queued factors and initial estimates to factor graph.
        temp_queue = deque([])
        while len(self.factor_queue):
            factor, estimates = self.factor_queue.popleft()
            self.graph.add(factor)
            for key, estimate in estimates:
                if not self.results.exists(key) and \
                        not self.initial_estimates.exists(key):
                    self.initial_estimates.insert(key, estimate)
                    if type(estimate) == gtsam.Pose3:
                        temp_queue.append(key)

        # Perform an iSAM2 incremental update.
        self.isam.update(self.graph, self.initial_estimates)
        self.results = self.isam.calculateEstimate()
        # print("*"*10)
        # if self.results.size() >= 3:
        #     print(self.results)
        # print("*"*10)
        self.poses_queue.extend(temp_queue)

        # Clear the graph and initial estimates.
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates.clear()

    ###################### Callbacks ######################

    def pose_callback(self, event=None):
        """Publish poses from the poses queue."""
        while len(self.poses_queue):
            key = self.poses_queue.popleft()
            pose_msg = self.process_pose_message(key)
            self.pose_publisher.publish(pose_msg)

    def trajectory_callback(self, event=None):
        """Publish the current optimized trajectory."""
        trajectory = []
        with self.optimizer_lock:
            poses = gtsam.utilities.allPose3s(self.results)
        keys = gtsam.KeyVector(poses.keys())
        for key in keys:
            try:
                pose_msg = self.process_pose_message(key)
                trajectory.append(pose_msg)
            except:
                rospy.logwarn("Could not access key in results")
        self.publish_traj(trajectory)

    def send_results(self, request):
        """Send a serialized string of the results."""
        serialized_str = self.results.serialize()
        return serialized_str

    def reset_results(self, request):
        """Send a serialized string of the results, reset
        the results and add the necessary prior factors.
        Used for testing purposes.
        """
        serialized_str = self.results.serialize()
        self.results.clear()
        self.isam = gtsam.ISAM2(gtsam.ISAM2Params())
        self.add_prior_factors()
        rospy.loginfo("Results cleared")
        return serialized_str

    ###################### Preprocessing helpers ######################

    def process_pose_message(self, key):
        """Produce a PoseStamped message of the most recent pose.

        Args:
            key: The key of the pose.
        Returns:
            pose_msg: A PoseStamped ROS message.
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"
        pose_estimate = self.results.atPose3(key)
        pose_msg.pose.position.x = pose_estimate.x()
        pose_msg.pose.position.y = pose_estimate.y()
        pose_msg.pose.position.z = pose_estimate.z()
        quaternion = pose_estimate.rotation().quaternion()
        pose_msg.pose.orientation.w = quaternion[0]
        pose_msg.pose.orientation.x = quaternion[1]
        pose_msg.pose.orientation.y = quaternion[2]
        pose_msg.pose.orientation.z = quaternion[3]
        return pose_msg

    def process_pose_cov_message(self, key):
        """Produce a PoseWithCovarianceStamped message
        of the most recent pose.

        Args:
            key: The key of the pose.
        Returns:
            pose_msg: A PoseWithCovarianceStamped ROS message.
        """
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"
        pose_estimate = self.results.atPose3(key)
        pose_msg.pose.pose.position.x = pose_estimate.x()
        pose_msg.pose.pose.position.y = pose_estimate.y()
        pose_msg.pose.pose.position.z = pose_estimate.z()
        quaternion = pose_estimate.rotation().quaternion()
        pose_msg.pose.pose.orientation.w = quaternion[0]
        pose_msg.pose.pose.orientation.x = quaternion[1]
        pose_msg.pose.pose.orientation.y = quaternion[2]
        pose_msg.pose.pose.orientation.z = quaternion[3]
        # Switch xyz and rpy covariance since format differs in GTSAM and ROS
        # Assume that off-diagonals are negligible - otherwise would need to modify.
        covariance = self.isam.marginalCovariance(key)
        rpy_cov, xyz_cov = covariance[:3, :3], covariance[3:, 3:]
        covariance[:3, :3] = xyz_cov
        covariance[3:, 3:] = rpy_cov
        pose_msg.pose.covariance = covariance.flatten()
        return pose_msg

    def publish_traj(self, trajectory):
        """Publish a Path message of the robot's trajectory."""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "world"
        path_msg.poses = trajectory
        self.traj_publisher.publish(path_msg)
