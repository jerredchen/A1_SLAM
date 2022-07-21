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
        lidar2d_topic = rospy.get_param('/lidar2d/topic')
        self.imu_pub = rospy.Publisher(imu_topic, HighState)
        self.lidar2d_pub = rospy.Publisher(lidar2d_topic, LaserScan)

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

        self.ranges_original = np.array(
            [np.inf]*4
            + [2, 2/np.sqrt(3), 1, 2/np.sqrt(3), 2]
            + [np.inf]*3
        )
        # Sleep to allow for subscribers to connect to the created topics.
        rospy.sleep(1)

    ############ Publisher/subscriber relevant functions ############

    def publish_multi_msgs(self, publishers, msgs, rates):
        """Publish toy data of multiple simultaneous sensors.
        Args:
            publishers: A list of rospy.Publisher objects, sorted from highest
                        publishing frequency to lowest publishing frequency.
            msgs:       A list of varying sensor messages to be published.
            rates:      A list of rates for each sensor (in Hz), sorted from
                        highest publishing frequency to lowest publishing frequency.
        """
        # Separate the highest frequency rate and the remaining lower frequencies.
        max_rate = rates[0]

        # Calculate the number of high frequency measurements to be published before
        # publishing a lower frequency measurement.
        pass_measurements = max_rate / np.array(rates[1:])
        ros_rate = rospy.Rate(max_rate)

        i = 0
        rospy.loginfo(f"{len(msgs)=}")
        while len(msgs):
            # Always publish the sensor at the fastest rate.
            msg = msgs.pop(0)
            msg.header.stamp = rospy.Time.now()
            publishers[0].publish(msg)
            rospy.loginfo(f"publishing {type(msg)=}")

            # Check to see if a lower frequency sensor should publish.
            for j, pass_measurement in enumerate(pass_measurements):
                if i == 0 or (i > 1 and ((i - 1) % pass_measurement == 0)):
                    msg = msgs.pop(0)
                    msg.header.stamp = rospy.Time.now()
                    rospy.loginfo(f"publishing {type(msg)=}")
                    publishers[j + 1].publish(msg)
                    break
            i += 1
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

    def generate_lidar_imu_data(self):
        """Generate LIDAR and IMU measurements when moving."""
        msgs = []
        j = 0
        for i in range(501):
            highstate = HighState()
            highstate.imu.gyroscope = [0, 0, 0]
            highstate.imu.accelerometer = [0, 0, 9.81]
            msgs.append(highstate)
            if i == 0 or (i > 1 and ((i - 1) % 50 == 0)):
                scan = LaserScan()
                scan.angle_increment = -np.pi/6
                scan.angle_min = np.pi
                scan.angle_max = -np.pi
                ranges = ((1.0 - 0.05*j) * self.ranges_original).tolist()
                scan.ranges = ranges
                msgs.append(scan)
                j += 1
        return msgs

    ####################### Integration tests #######################

    def test_lidar_with_imu(self):
        self.reset_results()
        self.reset_imu()
        rospy.sleep(0.5)

        self.publish_multi_msgs(
            [self.imu_pub, self.lidar2d_pub],
            self.generate_lidar_imu_data(),
            [50, 1]
        )

        # Generate the expected pose, velocity, and bias values.
        expected_values = gtsam.Values()
        expected_poses = gtsam.Values()
        for i in range(10):
            expected_values.insert(X(i), gtsam.Pose3(gtsam.Pose2(0.05*i, 0, 0)))
            expected_poses.insert(X(i), gtsam.Pose3(gtsam.Pose2(0.05*i, 0, 0)))
            expected_values.insert(V(i), gtsam.Point3(0.05, 0, 0))
        expected_values.insert(B(0), gtsam.imuBias.ConstantBias())

        # Sleep to finish obtaining the trajectory before clearing.
        rospy.sleep(1)

        # Obtain the actual poses.
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
