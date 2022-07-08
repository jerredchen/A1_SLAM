#!/usr/bin/env python

import gtsam
import numpy as np
import rospy
from unitree_legged_msgs.msg import HighState
from a1_slam.srv import ClearResults
from gtsam.symbol_shorthand import B, V, X
from gtsam.utils.test_case import GtsamTestCase
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan, PointCloud2


class PoseSLAM:
    def setUp(self):
        # rospy.init_node('test_node', anonymous=True)
        # rospy.loginfo("Initialized test_node")
        
        # Instantiate toy data publishers.
        self.imu_pub = None
        self.lidar2d_pub = None

        self.ranges_original = None
    
    ############ Publisher/subscriber relevant functions ############

    def publish_msgs(self, publisher, msgs, rate):
        """Publish toy data for testing.
        Args:
            publisher: a rospy.Publisher object to publish.
            msgs: A generator of messages to be published.
            rate: The rate to publish the messages, in Hz.
        """
        ros_rate = rospy.Rate(rate)
        for msg in msgs:
            publisher.publish(msg)
            ros_rate.sleep()

    ##################### Test data generators ######################

    def generate_no_acceleration_imu_data(self):
        """Generate IMU measurements when stationary."""
        for _ in range(500):
            highstate = HighState()
            highstate.header.stamp = rospy.Time.now()
            highstate.imu.accelerometer = [0, 0, 9.81]
            highstate.imu.gyroscope = [0, 0, 0]
            yield highstate

    def generate_acceleration_imu_data(self):
        """Generate IMU measurements when accelerating."""
        for _ in range(500):
            highstate = HighState()
            highstate.header.stamp = rospy.Time.now()
            highstate.imu.accelerometer = [1, 0, 9.81]
            highstate.imu.gyroscope = [0, 0, 0]
            yield highstate

    def generate_lidar_data(self):
        """Generate LIDAR measurements when moving."""
        for i in range(10):
            scan = LaserScan()
            scan.header.stamp = rospy.Time.now()
            scan.angle_increment = -np.pi/6
            scan.angle_min = np.pi
            scan.angle_max = -np.pi
            ranges = ((1.0 - 0.1*i) * self.ranges_original).tolist()
            scan.ranges = ranges
            yield scan
    
    ### Launch ###

    def launch(self):
        """Initialize and start the optimizer node."""
        rospy.init_node('node', anonymous=True)

        # Instantiate toy data publishers.
        imu_topic = 'imu'
        lidar2d_topic = 'lidar'
        self.imu_pub = rospy.Publisher(imu_topic, HighState)
        self.lidar2d_pub = rospy.Publisher(lidar2d_topic, LaserScan)

        rospy.logwarn(f"{str(LaserScan)}".split('.')[-1])
        rospy.logwarn(f"{str(HighState)}".split('.')[-1])
        rospy.logwarn(f"{type(self.imu_pub.type)}")

        self.ranges_original = np.array(
            [np.inf]*4
            + [2, 2/np.sqrt(3), 1, 2/np.sqrt(3), 2]
            + [np.inf]*3
        )

        self.publish_msgs(
            self.lidar2d_pub, self.generate_lidar_data(), 10)
        self.publish_msgs(
            self.imu_pub, self.generate_no_acceleration_imu_data(), 500)


if __name__ == '__main__':
    try:
        node = PoseSLAM()
        node.launch()
    except rospy.ROSInterruptException:
        pass