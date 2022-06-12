#!/usr/bin/env python

"""
Incrementally parsing ROSbag to obtain Imu data and produce Imu factor graph solution.
"""
import numpy as np
import rosbag
import rospy

import gtsam
from gtsam.symbol_shorthand import B, V, X
from geometry_msgs.msg import Pose, Pose2D
from sensor_msgs.msg import LaserScan, PointCloud2
from a1_slam.msg import HighState
from registration import icp_line, vanilla_ICP
from utils import A1_Plot


def main():
    """
    1. Instantiate the ros node
    2. If use_sensor == True:
            Call function which uses ros parameters to create init estimates and graph
            create_sensor_graph(graph)
    3. optimize_graph(graph, init_estimates, isam, pim=None, ...)
            optimize_graph will call sensor specific functions which will do its own processing
    """
    rospy.init_node('pose_slam_node', anonymous=True)
    use_imu = rospy.get_param("/use_imu")
    use_2dlidar = rospy.get_param("/use_2dlidar")
    use_depth = rospy.get_param("/use_depth")

    if use_imu:
        pass
    if use_2dlidar:
        pass
    if use_depth:
        pass


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass