#!/usr/bin/env python

import a1_slam
import gtsam
import numpy as np
import rospy
from a1_slam.msg import HighState
from a1_slam.srv import GtsamResults
from gtsam.symbol_shorthand import B, V, X
from gtsam.utils.test_case import GtsamTestCase
from geometry_msgs.msg import PoseStamped, Pose, Pose2D
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String


class TestPoseSLAM(GtsamTestCase):
    def setUp(self):
        rospy.init_node('test_node', anonymous=True)
        imu_topic = rospy.get_param('/imu_topic')
        lidar2d_topic = rospy.get_param('/2dlidar_topic')
        self.imu_pub = rospy.Publisher(imu_topic, HighState)
        self.lidar2d_pub = rospy.Publisher(lidar2d_topic, LaserScan)
        rospy.wait_for_service('send_results')
        self.send_results = rospy.ServiceProxy('send_results', GtsamResults)
        self.points_original = np.array([
                [-1.0, -1.0, 0.0, 1.0, 1.0, 1.0, 0.0, -1.0],
                [0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0, 1.0]
            ])

    def publish_msgs(self, publisher, msgs, rate):
        ros_rate = rospy.Rate(rate)
        for msg in msgs:
            publisher.publish(msg)
            ros_rate.sleep()
    
    def generate_stationary_imu_data(self):
        """Generate IMU measurements when stationary."""
        for i in range(500):
            highstate = HighState()
            highstate.header.stamp.secs = i / 100
            highstate.imu.accelerometer = [0, 0, 9.81]
            highstate.imu.gyroscope = [0, 0, 0]
            yield highstate

    def generate_acceleration_imu_data(self):
        """Generate IMU measurements when accelerating."""
        for i in range(500):
            highstate = HighState()
            highstate.header.stamp.secs = i / 100
            highstate.imu.accelerometer = [1, 0, 9.81]
            highstate.imu.gyroscope = [0, 0, 0]
            yield highstate

    def generate_lidar_data(self):
        """Generate LIDAR measurements when moving."""
        for i in range(5):
            scan = LaserScan()
            scan.angle_increment = np.pi/4
            scan.angle_min = -np.pi
            scan.angle_max = np.pi
            points = self.points_original - 0.05*i
            scan.ranges = (np.linalg.norm(points, axis=0)).tolist()
            yield scan

    def test_stationary_imu_poses(self):
        rospy.set_param('/use_imu', True)
        rospy.set_param('/use_2dlidar', False)
        rospy.set_param('/use_depth', False)
        self.publish_msgs(self.imu_pub, self.generate_stationary_imu_data(), 100)

        # Generate the expected pose, velocity, and bias values.
        expected = gtsam.Values()
        for i in range(5):
            expected.insert(X(i), gtsam.Pose3())
            expected.insert(V(i), gtsam.Point3(0, 0, 0))
        expected.insert(B(0), gtsam.imuBias.ConstantBias())

        # Obtain the actual pose, velocity, and bias values.
        serialized_str = self.send_results()
        actual = gtsam.Values()
        actual.deserialize(serialized_str)
        self.gtsamAssertEquals(actual, expected)

    def test_acceleration_imu_poses(self):
        rospy.set_param('/use_imu', True)
        rospy.set_param('/use_2dlidar', False)
        rospy.set_param('/use_depth', False)
        self.publish_msgs(self.imu_pub, self.generate_stationary_imu_data(), 100)

        # Generate the expected pose, velocity, and bias values.
        expected = gtsam.Values()
        for i in range(5):
            expected.insert(X(i), gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0.5*(i**2), 0.0, 0.0)))
            expected.insert(V(i), gtsam.Point3(i, 0, 0))
        expected.insert(B(0), gtsam.imuBias.ConstantBias())

        # Obtain the actual pose, velocity, and bias values.
        serialized_str = self.send_results()
        actual = gtsam.Values()
        actual.deserialize(serialized_str)
        self.gtsamAssertEquals(actual, expected, 1e-3)
    
    def test_lidar_poses(self):
        rospy.set_param('/use_imu', False)
        rospy.set_param('/use_2dlidar', True)
        rospy.set_param('/use_depth', False)
        self.publish_msgs(self.lidar2d_pub, self.generate_lidar_data(), 10)

        # Generate the expected LIDAR poses.
        expected = gtsam.Values()
        for i in range(5):
            expected.insert(X(i), gtsam.Pose2(0.05*i, 0.05*i, 0))

        # Obtain the actual poses.
        serialized_str = self.send_results()
        actual = gtsam.Values()
        actual.deserialize(serialized_str)
        self.gtsamAssertEquals(actual, expected, 1e-2)

if __name__ == '__main__':
    import rostest
    rostest.run('a1_slam', 'pose_slam_test', TestPoseSLAM)
