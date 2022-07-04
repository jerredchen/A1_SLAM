#!/usr/bin/env python

"""
Node which optimizes the factor graph using iSAM2.
"""
import numpy as np
import rospy

import gtsam
from unitree_legged_msgs.msg import HighState
from a1_slam.srv import GtsamResults, FinalResults, ClearResults
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

    def send_results_callback(self, request):
        serialized_str = self.results.serialize()
        return serialized_str

    def clear_results_callback(self, request):
        self.results.clear()
        rospy.logwarn("Results cleared")
        return self.results.size() == 0

    def optimize_graph_callback(self, request):

        factor_type = request.factor_type
        factor = None
        init_pose_estimate, init_vel_estimate, init_bias_estimate = None, None, None

        if factor_type == "PriorFactorPose3":
            factor = gtsam.PriorFactorPose3(
                0, gtsam.Pose3(), gtsam.noiseModel.Isotropic.Sigma(6, 1))
            init_pose_estimate = gtsam.Pose3()
        elif factor_type == "PriorFactorVector":
            factor = gtsam.PriorFactorVector(
                0, np.zeros((3,)), gtsam.noiseModel.Isotropic.Sigma(3, 1))
            init_pose_estimate = np.zeros((3,))
        elif factor_type == "PriorFactorConstantBias":
            factor = gtsam.PriorFactorConstantBias(
                0, gtsam.imuBias.ConstantBias(), gtsam.noiseModel.Isotropic.Sigma(6, 1))
            init_bias_estimate = gtsam.imuBias.ConstantBias()
        elif factor_type == "BetweenFactorPose3":
            factor = gtsam.BetweenFactorPose3(
                0, 0, gtsam.Pose3(), gtsam.noiseModel.Isotropic.Sigma(6, 1))
            init_pose_estimate = gtsam.Pose3()
        elif factor_type == "ImuFactor":
            initialized_pim = gtsam.PreintegratedImuMeasurements(
                gtsam.PreintegrationParams.MakeSharedU())
            factor = gtsam.ImuFactor(0, 0, 0, 0, 0, initialized_pim)
            init_pose_estimate = gtsam.Pose3()
        else:
            rospy.logerr(
                "Not a supported type of serialized factor.")
        factor.deserialize(request.factor)
        self.graph.add(factor)

        # Obtain the keys associated with the factor.
        if request.estimated_pose_key != -1:
            init_pose_estimate.deserialize(request.init_estimate)
            self.initial_estimates.insert(request.estimated_pose_key, init_pose_estimate)
        if request.estimated_vel_key != -1:
            init_vel_estimate = np.array(list(map(float, request.init_estimate.split())))
            self.initial_estimates.insert(request.estimated_vel_key, init_vel_estimate)
        if request.estimated_bias_key != -1:
            init_bias_estimate.deserialize(request.init_estimate)
            self.initial_estimates.insert(request.estimated_bias_key, init_bias_estimate)

        # Perform an iSAM2 incremental update.
        self.isam.update(self.graph, self.initial_estimates)
        self.results = self.isam.calculateEstimate()

        # Publish pose if new pose was added to trajectory.
        if request.estimated_pose_key != -1:
            # pose = self.results.atPose2()
            # rospy.loginfo(f"")
            pose_msg = self.process_pose_message(request.estimated_pose_key)
            self.publish_pose(pose_msg)

        # Clear the graph and initial estimates.
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates.clear()

        # Serialize the results to return.
        serialized_str = self.results.serialize()
        return serialized_str

    def create_trajectory_callback(self, event=None):
        trajectory = []
        poses = gtsam.utilities.allPose3s(self.results)
        keys = gtsam.KeyVector(poses.keys())
        for key in keys:
            try:
                pose_msg = self.process_pose_message(key)
                trajectory.append(pose_msg)
            except:
                rospy.logwarn("Could not access key in results")
        self.publish_traj(trajectory)

    def process_pose_message(self, key):
        """Publish a PoseStamped message of the most recent pose."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"
        pose_estimate = self.results.atPose3(key)
        pose_msg.pose.position.x = pose_estimate.x()
        pose_msg.pose.position.y = pose_estimate.y()
        pose_msg.pose.position.y = pose_estimate.z()
        quaternion = pose_estimate.rotation().quaternion()
        pose_msg.pose.orientation.w = quaternion[0]
        pose_msg.pose.orientation.x = quaternion[1]
        pose_msg.pose.orientation.y = quaternion[2]
        pose_msg.pose.orientation.z = quaternion[3]
        return pose_msg

    def publish_pose(self, pose_msg):
        self.pose_publisher.publish(pose_msg)

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
        rospy.Service('optimizer_service', GtsamResults,
                      self.optimize_graph_callback)
        rospy.Service('final_results_service', FinalResults,
                      self.send_results_callback)
        # Clear results service is only used for testing.
        rospy.Service('clear_results_service', ClearResults,
                      self.clear_results_callback)

        # Start a thread for publishing the trajectory._and_clear_
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
