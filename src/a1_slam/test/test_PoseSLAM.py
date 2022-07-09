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


class TestA1Slam(GtsamTestCase):
    def setUp(self):
        rospy.init_node('test_node', anonymous=True)
        rospy.loginfo("Initialized test_node")
        
        # Instantiate toy data publishers.
        imu_topic = rospy.get_param('/imu/topic')
        lidar2d_topic = rospy.get_param('/lidar2d/topic')
        self.imu_pub = rospy.Publisher(imu_topic, HighState)
        self.lidar2d_pub = rospy.Publisher(lidar2d_topic, LaserScan)

        # Instantiate relevant subscribers, attributes, and service to test results.
        rospy.Subscriber('/pose_estimate', PoseStamped, self.poses_callback)
        rospy.Subscriber('/traj_estimate', Path, self.traj_callback)
        rospy.wait_for_service('clear_results_service')
        self.clear_results = rospy.ServiceProxy(
            'clear_results_service', ClearResults)
        self.poses = gtsam.Values()
        self.traj = gtsam.Values()

        self.ranges_original = np.array(
            [np.inf]*4
            + [2, 2/np.sqrt(3), 1, 2/np.sqrt(3), 2]
            + [np.inf]*3
        )
        # Sleep to allow for subscribers to connect to the created topics.
        rospy.sleep(1)
    
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
    
    def publish_multi_msgs(self, publishers, msgs, rates):
        """Publish toy data of multiple simultaneous sensors.
        Args:
            publishers: A list of rospy.Publisher objects, sorted from highest
                        publishing frequency to lowest publishing frequency.
            msgs:       A generator of varying sensor messages to be published.
            rates:      A list of rates for each sensor, sorted from highest
                        publishing frequency to lowest publishing frequency.
        """
        # Separate the highest frequency rate and the remaining lower frequencies.
        max_rate = rates[0]

        # Calculate the number of high frequency measurements to be published before
        # publishing a lower frequency measurement.
        pass_measurements = max_rate / np.array(rates[1:])
        ros_rate = rospy.Rate(rates[0])
        for i, msg in enumerate(msgs):

            # Always publish the sensor at the fastest rate.
            publishers[0].publish(msg)

            # Check to see if a lower frequency sensor should publish.
            for j, pass_measurement in enumerate(pass_measurements):
                if i % pass_measurement == 0:
                    publishers[j + 1].publish(msg)
                    break
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
    
    def generate_lidar_imu_data(self):
        """Generate LIDAR and IMU measurements when moving."""
        for i in range(500):
            highstate = HighState()
            highstate.header.stamp = rospy.Time.now()
            highstate.imu.accelerometer = [0, 0, 9.81]
            highstate.imu.gyroscope = [0, 0, 0]
            yield highstate
            if i % 50 == 0:
                scan = LaserScan()
                scan.header.stamp = rospy.Time.now()
                scan.angle_increment = -np.pi/6
                scan.angle_min = np.pi
                scan.angle_max = -np.pi
                ranges = ((1.0 - 0.1*i) * self.ranges_original).tolist()
                scan.ranges = ranges
                yield scan

    ####################### Integration tests #######################

    def test_stationary_imu_poses(self):
        rospy.set_param('/use_imu', True)
        rospy.set_param('/use_2dlidar', False)
        self.publish_msgs(self.imu_pub, self.generate_no_acceleration_imu_data(), 100)

        # Generate the expected pose, velocity, and bias values.
        expected = gtsam.Values()
        for i in range(5):
            expected.insert(X(i), gtsam.Pose3())
            expected.insert(V(i), gtsam.Point3(0, 0, 0))
        expected.insert(B(0), gtsam.imuBias.ConstantBias())

        # Sleep to finish obtaining the trajectory before clearing.
        rospy.sleep(1)

        # Obtain the actual pose, velocity, and bias values.
        response = self.clear_results()
        actual = gtsam.Values()
        actual.deserialize(response.results)

        self.gtsamAssertEquals(actual, expected)
        self.gtsamAssertEquals(self.poses, expected, 1e-2)
        self.gtsamAssertEquals(self.traj, expected, 1e-2)

    def test_acceleration_imu_poses(self):
        rospy.set_param('/use_imu', True)
        rospy.set_param('/use_2dlidar', False)
        self.publish_msgs(self.imu_pub, self.generate_acceleration_imu_data(), 100)

        # Generate the expected pose, velocity, and bias values.
        expected = gtsam.Values()
        for i in range(5):
            expected.insert(X(i), gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0.5*(i**2), 0.0, 0.0)))
            expected.insert(V(i), gtsam.Point3(i, 0, 0))
        expected.insert(B(0), gtsam.imuBias.ConstantBias())

        # Sleep to finish obtaining the trajectory before clearing.
        rospy.sleep(1)

        # Obtain the actual pose, velocity, and bias values.
        response = self.clear_results()
        actual = gtsam.Values()
        actual.deserialize(response.results)

        self.gtsamAssertEquals(actual, expected, 1e-3)
        self.gtsamAssertEquals(self.poses, expected, 1e-3)
        self.gtsamAssertEquals(self.traj, expected, 1e-3)
        self.poses.clear()
        self.traj.clear()

    def test_lidar_poses(self):
        rospy.set_param('/use_imu', False)
        rospy.set_param('/use_2dlidar', True)
        self.publish_msgs(
            self.lidar2d_pub, self.generate_lidar_data(), 10)

        # Generate the expected LIDAR poses.
        expected = gtsam.Values()
        for i in range(10):
            expected.insert(X(i), gtsam.Pose3(gtsam.Pose2(0.1*i, 0, 0)))

        # Sleep to finish obtaining the trajectory before clearing.
        rospy.sleep(1)

        # Obtain the actual poses.
        response = self.clear_results()
        actual = gtsam.Values()
        actual.deserialize(response.results)

        self.gtsamAssertEquals(actual, expected, 1e-2)
        self.gtsamAssertEquals(self.poses, expected, 1e-2)
        self.gtsamAssertEquals(self.traj, expected, 1e-2)
        self.poses.clear()
        self.traj.clear()
    
    def test_lidar_with_imu(self):
        rospy.set_param('/use_imu', True)
        rospy.set_param('/use_2dlidar', True)
        self.publish_multi_msgs(
            [self.imu_pub, self.lidar2d_pub],
            self.generate_lidar_imu_data(),
            [500, 10]
        )

        # Generate the expected LIDAR poses.
        expected = gtsam.Values()
        for i in range(10):
            expected.insert(X(i), gtsam.Pose3(gtsam.Pose2(0.1*i, 0, 0)))

        # Sleep to finish obtaining the trajectory before clearing.
        rospy.sleep(1)

        # Obtain the actual poses.
        response = self.clear_results()
        actual = gtsam.Values()
        actual.deserialize(response.results)

        self.gtsamAssertEquals(actual, expected, 1e-2)
        self.gtsamAssertEquals(self.poses, expected, 1e-2)
        self.gtsamAssertEquals(self.traj, expected, 1e-2)
        self.poses.clear()
        self.traj.clear()


if __name__ == '__main__':
    import rostest
    rostest.run('a1_slam', 'pose_slam_test', TestA1Slam)
