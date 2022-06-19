#!/usr/bin/env python

"""
Incrementally performing PoseSLAM using one or more sensors on the A1.
"""
import numpy as np
import rospy

import gtsam
from a1_slam.msg import HighState
from a1_slam.srv import GtsamResults
from collections import deque
from gtsam.symbol_shorthand import B, V, X
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from sensors import IMU_helpers, LIDAR_2D_helpers


N_GRAVITY = [0, 0, -9.81]


class PoseSlamNode():
    def __init__(self):

        # Instantiate publisher attributes.
        self.pose_publisher = rospy.Publisher("pose_estimate", PoseStamped, queue_size=5)

        # Instantiate factor graph and optimizer attributes.
        self.state_index = 0
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()
        self.results = gtsam.Values()
        self.isam = gtsam.ISAM2(gtsam.ISAM2Params())

        # Instantiate IMU related attributes.
        self.pim = gtsam.PreintegratedImuMeasurements(
            gtsam.PreintegrationParams.MakeSharedU())
        self.timestamp = 0

        # Instantiate 2D LIDAR related attributes.
        self.icp_noise = gtsam.noiseModel.Diagonal.Sigmas(np.ones((3,)))
        self.submap_scans = deque([], rospy.get_param('/lidar_submap_length'))

    def send_results_callback(self, request):
        serialized_str = self.results.serialize()
        self.sent_results = True
        return serialized_str

    def imu_callback(self, msg):
        """If imu_callback is called:
            - only performs preintegration
        """
        curr_timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
        self.pim = IMU_helpers.preintegrate_measurement(
            msg,
            curr_timestamp - self.timestamp,
            self.pim,
            N_GRAVITY
        )
        self.timestamp = curr_timestamp

    def lidar_callback(self, msg):
        """If lidar_callback is called:
            - will perform registration with previous scan
            - will create lidar odometry factor
            - will create imu factor (if use_imu == True)
            - will call isam to optimize graph
                - after isam called, publish pose, trajectory
            - maybe additionally add further registration constraints?
        """
        rospy.loginfo(f"lidar callback, receiving {msg.header.seq=}")
        scan = LIDAR_2D_helpers.preprocess_measurement(msg)
        rospy.loginfo(f"{scan=}")
        if len(self.submap_scans) == 0:
            self.submap_scans.append(scan)
            return
        self.graph, self.initial_estimates = LIDAR_2D_helpers.add_lidar_factor(
            self.state_index-1,
            self.state_index,
            self.submap_scans[-1],
            scan,
            self.icp_noise,
            self.graph,
            self.initial_estimates,
            self.results,
            "vanilla"
        )
        self.submap_scans.append(scan)
        self.optimize_graph()

    def optimize_graph(self):

        rospy.loginfo("optimizing factor graph")

        prev_results_size = self.results.size()

        # Perform an iSAM2 incremental update.
        self.isam.update(self.graph, self.initial_estimates)
        self.results = self.isam.calculateEstimate()

        # Publish pose if new pose was added to trajectory.
        if self.results.size() > prev_results_size:
            self.publish_pose()

        # Clear the graph and initial estimates, and update the state index.
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates.clear()
        self.state_index += 1

    def publish_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "body"
        if rospy.get_param('use_2dlidar'):
            pose_estimate = self.results.atPose2(X(self.state_index))
            pose_msg.pose.position.x = pose_estimate.x()
            pose_msg.pose.position.y = pose_estimate.y()
            quaternion = gtsam.Rot3.Ypr(pose_estimate.theta(), 0, 0).quaternion()
        else:
            pose_estimate = self.results.atPose3(X(self.state_index))
            pose_msg.pose.position.x = pose_estimate.x()
            pose_msg.pose.position.y = pose_estimate.y()
            pose_msg.pose.position.y = pose_estimate.z()
            quaternion = gtsam.rotation().quaternion()
        pose_msg.pose.orientation.x = quaternion[1]
        pose_msg.pose.orientation.y = quaternion[2]
        pose_msg.pose.orientation.z = quaternion[3]
        pose_msg.pose.orientation.w = quaternion[0]
        self.pose_publisher.publish(pose_msg)

    def perform_pose_slam(self):
        """
        1. Instantiate the ros node
        2. If use_sensor == True:
                Call function which uses ros parameters to create init estimates and graph
                create_sensor_graph(graph)
        3. optimize_graph(graph, init_estimates, isam, pim=None, ...)
                optimize_graph will call sensor specific functions which will do its own processing
        """
        rospy.init_node('pose_slam_node', anonymous=True)
        rospy.loginfo("Initialized pose_slam_node")

        # Parse the rosparams to identify which sensors to use.
        use_imu = rospy.get_param("/use_imu")
        use_2dlidar = rospy.get_param("/use_2dlidar")
        use_depth = rospy.get_param("/use_depth")
        if not any([use_imu, use_2dlidar, use_depth]):
            raise RuntimeError(
                "No sensors selected. Please select at least one of the sensors in the YAML file.")

        if use_imu:
            self.graph, self.initial_estimates = IMU_helpers.create_imu_graph_and_params(
                self.graph, self.initial_estimates)

            rospy.Subscriber(rospy.get_param("/imu_topic"),
                             HighState,
                             callback=self.imu_callback)

        if use_2dlidar:
            self.graph, self.initial_estimates, self.icp_noise = LIDAR_2D_helpers.create_lidar_graph_and_params(
                self.graph, self.initial_estimates)

            rospy.Subscriber(rospy.get_param("/2dlidar_topic"),
                             LaserScan,
                             callback=self.lidar_callback)
        if use_depth:
            pass

        rospy.Service('send_results', GtsamResults, self.send_results_callback)
        rospy.loginfo("Initialized results service")

        # Instantiate the iSAM2 parameters to create the iSAM2 object.
        parameters = gtsam.ISAM2Params()
        self.isam = gtsam.ISAM2(parameters)

        # Perform an iSAM2 update to populate the results and publish the start pose.
        self.isam.update(self.graph, self.initial_estimates)
        self.results = self.isam.calculateEstimate()
        self.publish_pose()

        # Clear the graph and initial estimates, and update the state index.
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates.clear()
        self.state_index += 1

        rospy.spin()


if __name__ == "__main__":
    try:
        node = PoseSlamNode()
        node.perform_pose_slam()
    except rospy.ROSInterruptException:
        pass
