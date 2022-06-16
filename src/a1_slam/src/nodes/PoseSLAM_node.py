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

# Instantiate publisher global variables
pose_publisher = None

# Instantiate actor graph and optimizer global variables
state_index = 0
graph = gtsam.NonlinearFactorGraph()
initial_estimates = gtsam.Values()
results = gtsam.Values()
isam = None

# Instantiate IMU global variables
pim = None
timestamp = 0

# Instantiate 2D LIDAR global variables
icp_noise = None
submap_scans = deque([], rospy.get_param('/lidar_submap_length'))


def send_results_callback(request):
    global results
    return results


def imu_callback(msg):
    """If imu_callback is called:
        - only performs preintegration
    """
    global timestamp
    curr_timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
    pim = IMU_helpers.preintegrate_measurement(
        msg,
        curr_timestamp - timestamp,
        pim,
        N_GRAVITY
    )
    timestamp = curr_timestamp


def lidar_callback(msg):
    """If lidar_callback is called:
        - will perform registration with previous scan
        - will create lidar odometry factor
        - will create imu factor (if use_imu == True)
        - will call isam to optimize graph
            - after isam called, publish pose, trajectory
        - maybe additionally add further registration constraints?
    """
    global state_index
    scan = LIDAR_2D_helpers.preprocess_measurement(msg)
    if len(submap_scans) == 0:
        submap_scans.append(scan)
        return
    graph, initial_estimates = LIDAR_2D_helpers.add_lidar_factor(
        state_index-1,
        state_index,
        submap_scans[-1],
        scan,
        icp_noise,
        graph,
        initial_estimates
    )
    submap_scans.append(scan)
    optimize_graph()


def optimize_graph():
    global pose_publisher, state_index, graph, initial_estimates, isam, results

    prev_results_size = results.size()

    # Perform an iSAM2 incremental update.
    isam.update(graph, initial_estimates)
    results = isam.calculateEstimate()
    if results.size() > prev_results_size:
        if rospy.get_param('use_2dlidar'):
            pose_estimate = results.atPose2(X(state_index))
            pose_msg = Pose2D()
            pose_msg.x = pose_estimate.x()
            pose_msg.y = pose_estimate.y()
            pose_msg.theta = pose_estimate.theta()
            pose_publisher.publish(pose_msg)
        else:
            pose_estimate = results.atPose3(X(state_index))
            pose_msg = Pose()
            pose_msg.position.x = pose_estimate.x()
            pose_msg.position.y = pose_estimate.y()
            pose_msg.position.y = pose_estimate.z()
            quaternion = gtsam.rotation().quaternion()
            pose_msg.orientation.x = quaternion[1]
            pose_msg.orientation.y = quaternion[2]
            pose_msg.orientation.z = quaternion[3]
            pose_msg.orientation.w = quaternion[0]
            pose_publisher.publish(pose_msg)

    # Clear the graph and initial estimates.
    graph = gtsam.NonlinearFactorGraph()
    initial_estimates.clear()
    state_index += 1


def perform_pose_slam():
    """
    1. Instantiate the ros node
    2. If use_sensor == True:
            Call function which uses ros parameters to create init estimates and graph
            create_sensor_graph(graph)
    3. optimize_graph(graph, init_estimates, isam, pim=None, ...)
            optimize_graph will call sensor specific functions which will do its own processing
    """
    global graph, initial_estimates, isam, icp_noise, pose_publisher
    rospy.init_node('pose_slam_node', anonymous=True)

    # Parse the rosparams to identify which sensors to use.
    use_imu = rospy.get_param("/use_imu")
    use_2dlidar = rospy.get_param("/use_2dlidar")
    use_depth = rospy.get_param("/use_depth")
    if not any([use_imu, use_2dlidar, use_depth]):
        raise RuntimeError(
            "No sensors selected. Please select at least one of the sensors in the YAML file.")

    if use_imu:
        graph, initial_estimates = IMU_helpers.create_imu_graph_and_params(
            graph, initial_estimates)
        pose_publisher = rospy.Publisher("Pose", Pose)

        # Instantiate the IMU subscriber
        imu_topic = rospy.get_param("/imu_topic")
        rospy.Subscriber(imu_topic, HighState)

    if use_2dlidar:
        graph, initial_estimates, icp_noise = LIDAR_2D_helpers.create_lidar_graph_and_params(
            graph, initial_estimates)
        pose_publisher = rospy.Publisher("Pose", Pose2D)

        # Instantiate the LIDAR subscriber
        lidar_topic = rospy.get_param("/lidar_topic")
        rospy.Subscriber(lidar_topic, LaserScan)

    if use_depth:
        pass

    rospy.Service('send_results', GtsamResults, send_results_callback)

    # Instantiate the iSAM2 parameters to create the iSAM2 object.
    parameters = gtsam.ISAM2Params()
    isam = gtsam.ISAM2(parameters)

    rospy.spin()


if __name__ == "__main__":
    try:
        perform_pose_slam()
    except rospy.ROSInterruptException:
        pass
