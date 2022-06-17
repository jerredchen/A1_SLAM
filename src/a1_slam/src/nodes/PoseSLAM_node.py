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
from geometry_msgs.msg import Pose, Pose2D
from sensor_msgs.msg import LaserScan, PointCloud2
from sensors import IMU_helpers, LIDAR_2D_helpers


N_GRAVITY = [0, 0, -9.81]

class PoseSlamNode():
    def __init__(self):

        # Instantiate publisher global variables
        self.pose_publisher = None

        # Instantiate actor graph and optimizer global variables
        self.state_index = 1
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()
        self.results = gtsam.Values()
        self.isam = None

        # Instantiate IMU global variables
        self.pim = None
        self.timestamp = 0
        # Instantiate 2D LIDAR global variables
        self.icp_noise = None
        self.submap_scans = deque([], rospy.get_param('/lidar_submap_length'))


    def send_results_callback(self, request):
        serialized_str = self.results.serialize()
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
        if self.results.size() > prev_results_size:
            if rospy.get_param('use_2dlidar'):
                pose_estimate = self.results.atPose2(X(self.state_index))
                pose_msg = Pose2D()
                pose_msg.x = pose_estimate.x()
                pose_msg.y = pose_estimate.y()
                pose_msg.theta = pose_estimate.theta()
                self.pose_publisher.publish(pose_msg)
            else:
                pose_estimate = self.results.atPose3(X(self.state_index))
                pose_msg = Pose()
                pose_msg.position.x = pose_estimate.x()
                pose_msg.position.y = pose_estimate.y()
                pose_msg.position.y = pose_estimate.z()
                quaternion = gtsam.rotation().quaternion()
                pose_msg.orientation.x = quaternion[1]
                pose_msg.orientation.y = quaternion[2]
                pose_msg.orientation.z = quaternion[3]
                pose_msg.orientation.w = quaternion[0]
                self.pose_publisher.publish(pose_msg)

        # Clear the graph and initial estimates.
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates.clear()
        self.state_index += 1


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
            self.pose_publisher = rospy.Publisher("Pose", Pose, queue_size=5)

            # Instantiate the IMU subscriber
            imu_topic = rospy.get_param("/imu_topic")
            rospy.Subscriber(imu_topic, HighState, callback=self.imu_callback)

        if use_2dlidar:
            self.graph, self.initial_estimates, self.icp_noise = LIDAR_2D_helpers.create_lidar_graph_and_params(
                self.graph, self.initial_estimates)
            self.pose_publisher = rospy.Publisher("Pose", Pose2D, queue_size=5)

            # Instantiate the LIDAR subscriber
            lidar_topic = rospy.get_param("/2dlidar_topic")
            rospy.Subscriber(lidar_topic, LaserScan, callback=self.lidar_callback)

        if use_depth:
            pass

        rospy.Service('send_results', GtsamResults, self.send_results_callback)
        rospy.loginfo("Initialized results service")

        # Instantiate the iSAM2 parameters to create the iSAM2 object.
        parameters = gtsam.ISAM2Params()
        self.isam = gtsam.ISAM2(parameters)

        # Perform an iSAM2 incremental update.
        self.isam.update(self.graph, self.initial_estimates)
        self.results = self.isam.calculateEstimate()

        # Clear the graph and initial estimates.
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates.clear()

        rospy.spin()


if __name__ == "__main__":
    try:
        node = PoseSlamNode()
        node.perform_pose_slam()
    except rospy.ROSInterruptException:
        pass
