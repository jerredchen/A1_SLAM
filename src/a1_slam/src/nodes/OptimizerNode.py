#!/usr/bin/env python

"""
Node which optimizes the factor graph using iSAM2.
"""
import numpy as np
import rospy

import gtsam
from unitree_legged_msgs.msg import HighState
from a1_slam.srv import GtsamResults, FinalResults
from gtsam.symbol_shorthand import B, V, X
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan, PointCloud2


class OptimizerNode():
    def __init__(self):

        # Instantiate publisher attributes.
        self.pose_publisher = rospy.Publisher(
            "/pose_estimate", PoseStamped, queue_size=5
        )
        self.traj_publisher = rospy.Publisher(
            "/traj_estimate", Path, queue_size=5
        )

        # Instantiate factor graph and optimizer attributes.
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()
        self.results = gtsam.Values()
        self.isam = gtsam.ISAM2(gtsam.ISAM2Params())

    def send_and_clear_results_callback(self, request):
        serialized_str = self.results.serialize()
        self.results.clear()
        return serialized_str

    def optimize_graph_callback(self, request):
        rospy.loginfo("optimizing factor graph")

        factor_type = request.factor_type
        factor, init_estimate = None, None

        if factor_type == "PriorFactorPose2":
            factor = gtsam.PriorFactorPose2(
                0, gtsam.Pose2(), gtsam.noiseModel.Isotropic.Sigma(3, 1))
            init_estimate = gtsam.Pose2()
        elif factor_type == "BetweenFactorPose2":
            factor = gtsam.BetweenFactorPose2(
                0, 0, gtsam.Pose2(), gtsam.noiseModel.Isotropic.Sigma(3, 1))
            init_estimate = gtsam.Pose2()
        elif factor_type == "PriorFactorPose3":
            factor = gtsam.PriorFactorPose3(
                0, gtsam.Pose3(), gtsam.noiseModel.Isotropic.Sigma(6, 1))
            init_estimate = gtsam.Pose2()
        elif factor_type == "BetweenFactorPose3":
            factor = gtsam.BetweenFactorPose3(
                0, 0, gtsam.Pose3(), gtsam.noiseModel.Isotropic.Sigma(6, 1))
            init_estimate = gtsam.Pose3()
        else:
            rospy.logerr(
                "Not currently supported for other than Pose2 or Pose3")
        factor.deserialize(request.factor)
        self.graph.add(factor)

        # Obtain the keys associated with the factor.
        key_vector = factor.keys()
        if len(request.init_estimate) != 0:
            init_estimate.deserialize(request.init_estimate)
            
            self.initial_estimates.insert(key_vector[-1], init_estimate)

        prev_results_size = self.results.size()

        # Perform an iSAM2 incremental update.
        self.isam.update(self.graph, self.initial_estimates)
        self.results = self.isam.calculateEstimate()

        # Publish pose if new pose was added to trajectory.
        if self.results.size() > prev_results_size and (
            "Pose2" in factor_type or "Pose3" in factor_type
        ):
            pose_msg = self.process_pose_message(key_vector[-1])
            self.publish_pose(pose_msg)

        # Clear the graph and initial estimates, and update the state index.
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates.clear()

        # Serialize the results to return.
        serialized_str = self.results.serialize()

        return serialized_str

    def create_trajectory_callback(self, event=None):
        trajectory = []
        if rospy.get_param("/use_2dlidar"):
            poses = gtsam.utilities.allPose2s(self.results)
        else:
            poses = gtsam.utilities.allPose3s(self.results)
        keys = gtsam.KeyVector(poses.keys())
        for key in keys:
            trajectory.append(self.process_pose_message(key))
        self.publish_traj(trajectory)

    def process_pose_message(self, key):
        """Publish a PoseStamped message of the most recent pose."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"
        if rospy.get_param('use_2dlidar'):
            pose_estimate = self.results.atPose2(key)
            pose_msg.pose.position.x = pose_estimate.x()
            pose_msg.pose.position.y = pose_estimate.y()
            quaternion = gtsam.Rot3.Ypr(
                pose_estimate.theta(), 0, 0).quaternion()
        else:
            pose_estimate = self.results.atPose3(key)
            pose_msg.pose.position.x = pose_estimate.x()
            pose_msg.pose.position.y = pose_estimate.y()
            pose_msg.pose.position.y = pose_estimate.z()
            quaternion = gtsam.rotation().quaternion()
        pose_msg.pose.orientation.w = quaternion[0]
        pose_msg.pose.orientation.x = quaternion[1]
        pose_msg.pose.orientation.y = quaternion[2]
        pose_msg.pose.orientation.z = quaternion[3]
        return pose_msg
    
    def publish_pose(self, pose_msg):
        self.pose_publisher.publish(pose_msg)
        rospy.loginfo(f"publishing {pose_msg.header.seq=}")

    def publish_traj(self, trajectory):
        """Publish a Path message of the robot's trajectory."""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "world"
        path_msg.poses = trajectory
        self.traj_publisher.publish(path_msg)

    def launch_optimizer_node(self):
        """Initialize and start the optimizer node."""
        rospy.init_node('optimizer_node', anonymous=True)

        # Start the necessary services for optimizing and sending results.
        rospy.Service('optimizer_service', GtsamResults, self.optimize_graph_callback)
        rospy.Service('final_results_service',
                      FinalResults, self.send_and_clear_results_callback)

        # Start a thread for publishing the trajectory.
        rospy.Timer(rospy.Duration(0.5), self.create_trajectory_callback)

        # Instantiate the iSAM2 parameters to create the iSAM2 object.
        parameters = gtsam.ISAM2Params()
        self.isam = gtsam.ISAM2(parameters)

        rospy.spin()


if __name__ == "__main__":
    try:
        node = OptimizerNode()
        node.launch_optimizer_node()
    except rospy.ROSInterruptException:
        pass
