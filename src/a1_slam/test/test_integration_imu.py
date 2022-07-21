#!/usr/bin/env python

import gtsam
import numpy as np
import rospy
from unitree_legged_msgs.msg import HighState
from a1_slam.srv import GetResults
from gtsam.symbol_shorthand import B, V, X
from gtsam.utils.test_case import GtsamTestCase
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan, PointCloud2


class TestA1Slam(GtsamTestCase):
    def setUp(self):
        rospy.init_node('test_node', anonymous=True)
        rospy.loginfo("Initialized test_node")
        
        # Instantiate toy data publishers.
        imu_topic = rospy.get_param('/imu/topic')
        self.imu_pub = rospy.Publisher(imu_topic, HighState)

        # Instantiate relevant subscribers, services, and attributes to test results.
        rospy.Subscriber('/pose_estimate', PoseStamped, self.poses_callback)
        rospy.Subscriber('/traj_estimate', Path, self.traj_callback)
        rospy.wait_for_service('get_results_service')
        self.send_results = rospy.ServiceProxy(
            'get_results_service', GetResults)
        rospy.wait_for_service('reset_results_service')
        self.reset_results = rospy.ServiceProxy(
            'reset_results_service', GetResults)
        rospy.wait_for_service('reset_imu_service')
        self.reset_imu = rospy.ServiceProxy(
            'reset_imu_service', GetResults)
        self.poses = gtsam.Values()
        self.traj = gtsam.Values()

        # Sleep to allow for subscribers to connect to the created topics.
        rospy.sleep(1)

    ############ Publisher/subscriber relevant functions ############

    def publish_msgs(self, publisher, msgs, rate):
        """Publish toy data for testing.
        Args:
            publisher:  A rospy.Publisher object to publish.
            msgs:       A generator of messages to be published.
            rate:       The rate to publish the messages, in Hz.
        """
        ros_rate = rospy.Rate(rate)
        for msg in msgs:
            publisher.publish(msg)
            ros_rate.sleep()

    def process_pose_msg(self, msg: PoseStamped):
        """Convert PoseStamped message into GTSAM pose."""
        rotation = gtsam.Rot3.Quaternion(
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
        )
        translation = gtsam.Point3(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )
        pose = gtsam.Pose3(rotation, translation)
        return pose

    def poses_callback(self, msg: PoseStamped):
        pose = self.process_pose_msg(msg)
        self.poses.insert(X(self.poses.size()), pose)
    
    def traj_callback(self, msg: Path):
        new_traj = gtsam.Values()
        for pose_msg in msg.poses:
            pose = self.process_pose_msg(pose_msg)
            new_traj.insert(X(new_traj.size()), pose)
        self.traj = new_traj

    ##################### Test data generators ######################

    def generate_no_acceleration_imu_data(self):
        """Generate IMU measurements when stationary."""
        for _ in range(501):
            highstate = HighState()
            highstate.header.stamp = rospy.Time.now()
            highstate.imu.accelerometer = [0, 0, 9.81]
            highstate.imu.gyroscope = [0, 0, 0]
            yield highstate

    def generate_acceleration_imu_data(self):
        """Generate IMU measurements when accelerating."""
        for _ in range(501):
            highstate = HighState()
            highstate.header.stamp = rospy.Time.now()
            highstate.imu.accelerometer = [1, 0, 9.81]
            highstate.imu.gyroscope = [0, 0, 0]
            yield highstate

    ####################### Integration tests #######################

    def test_stationary_imu_poses(self):
        self.reset_results()
        self.reset_imu()

        self.publish_msgs(self.imu_pub, self.generate_no_acceleration_imu_data(), 100)

        # Generate the expected pose, velocity, and bias values.
        expected_values = gtsam.Values()
        expected_poses = gtsam.Values()
        for i in range(6):
            expected_values.insert(X(i), gtsam.Pose3())
            expected_poses.insert(X(i), gtsam.Pose3())
            expected_values.insert(V(i), gtsam.Point3(0, 0, 0))
        expected_values.insert(B(0), gtsam.imuBias.ConstantBias())

        # Sleep to finish obtaining the trajectory before clearing.
        rospy.sleep(1)

        # Obtain the actual pose, velocity, and bias values.
        response = self.send_results()
        actual = gtsam.Values()
        actual.deserialize(response.results)

        self.gtsamAssertEquals(actual, expected_values, 5e-3)
        self.gtsamAssertEquals(self.poses, expected_poses, 5e-3)
        self.gtsamAssertEquals(self.traj, expected_poses, 5e-3)
        self.poses.clear()
        self.traj.clear()

    def test_acceleration_imu_poses(self):
        self.reset_results()
        self.reset_imu()

        self.publish_msgs(self.imu_pub, self.generate_acceleration_imu_data(), 100)

        # Generate the expected pose, velocity, and bias values.
        expected_values = gtsam.Values()
        expected_poses = gtsam.Values()
        for i in range(6):
            expected_values.insert(X(i), gtsam.Pose3(gtsam.Pose2(0.5*(i**2), 0.0, 0.0)))
            expected_poses.insert(X(i), gtsam.Pose3(gtsam.Pose2(0.5*(i**2), 0.0, 0.0)))
            expected_values.insert(V(i), gtsam.Point3(i, 0, 0))
        expected_values.insert(B(0), gtsam.imuBias.ConstantBias())

        # Sleep to finish obtaining the trajectory before clearing.
        rospy.sleep(1)

        # Obtain the actual pose, velocity, and bias values.
        response = self.send_results()
        actual = gtsam.Values()
        actual.deserialize(response.results)

        self.gtsamAssertEquals(actual, expected_values, 5e-3)
        self.gtsamAssertEquals(self.poses, expected_poses, 5e-3)
        self.gtsamAssertEquals(self.traj, expected_poses, 5e-3)
        self.poses.clear()
        self.traj.clear()


if __name__ == '__main__':
    import rostest
    rostest.run('a1_slam', 'pose_slam_test', TestA1Slam)
