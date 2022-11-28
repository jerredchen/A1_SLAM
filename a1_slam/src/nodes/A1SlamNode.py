#!/usr/bin/env python

import rospy
from a1_slam.srv import GetResults
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from unitree_legged_msgs.msg import HighState

from optimization.optimizer import Optimizer
from sensors.imu import ImuWrapper
from sensors.lidar2D import Lidar2DWrapper
from sensors.lidar3D import Lidar3DWrapper
from sensors.depth import DepthWrapper


class A1SlamNode():

    def launch_node(self):
        """Initialize and start the A1 SLAM node."""
        rospy.init_node('a1_slam_node', anonymous=True)

        # Check which sensors are selected to be used for SLAM.
        use_imu = rospy.get_param('/use_imu')
        use_2dlidar = rospy.get_param('/use_2dlidar')
        use_3dlidar = rospy.get_param('/use_3dlidar')
        use_depth = rospy.get_param('/use_depth')
        check_sensors = (use_imu, use_2dlidar, use_3dlidar, use_depth)
        if not any(check_sensors):
            rospy.logerr("No sensors were selected.")

        # Create the optimizer with prior factors to add to the factor graph
        optimizer = Optimizer()
        optimizer.add_prior_factors()

        # Create an Imu wrapper class that subscribes to imu.
        imu = None
        if use_imu:
            imu = ImuWrapper(optimizer)
            imu.initialize_params()
            imu_topic = rospy.get_param('/imu/topic')
            imu_only = not use_2dlidar and not use_depth
            if rospy.get_param('/imu/use_custom_unitree'):
                rospy.Subscriber(
                    imu_topic, HighState, imu.imu_unitree_callback, callback_args=imu_only)
            else:
                rospy.Subscriber(
                    imu_topic, Imu, imu.imu_ros_callback, callback_args=imu_only)

        # Create a Lidar2D wrapper class that subscribes to laser scans.
        if use_2dlidar:
            lidar2d = Lidar2DWrapper(optimizer)
            lidar2d.initialize_params()
            lidar2d_topic = rospy.get_param('/lidar2d/topic')
            rospy.Subscriber(
                lidar2d_topic,
                LaserScan,
                lidar2d.lidar_callback,
                callback_args=imu
            )

        # Create a Lidar3D wrapper class that subscribes to laser scans.
        if use_3dlidar:
            lidar3d = Lidar3DWrapper(optimizer)
            lidar3d.initialize_params()
            lidar3d_topic = rospy.get_param('/lidar3d/topic')
            rospy.Subscriber(
                lidar3d_topic,
                PointCloud2,
                lidar3d.lidar_callback,
                callback_args=imu
            )

        # Create a Depth wrapper class that subscribes to laser scans.
        if use_depth:
            depth = DepthWrapper(optimizer)
            depth.initialize_params()
            depth_topic = rospy.get_param('/depth/topic')
            rospy.Subscriber(
                depth_topic,
                PointCloud2,
                depth.depth_callback,
                callback_args=imu
            )

        optimizer.optimize()

        # Create a pose publisher that publishes the pose from the results.
        rospy.Timer(rospy.Duration(0.05), optimizer.pose_callback)

        # Create a trajectory publisher that publishes the trajectory of results.
        rospy.Timer(rospy.Duration(1.0), optimizer.trajectory_callback)

        # Start the necessary service for sending results.
        rospy.Service('get_results_service', GetResults,
                      optimizer.send_results)

        # Start services that are only used for ros integration testing.
        if rospy.get_param('/testing'):
            rospy.Service('reset_results_service', GetResults,
                        optimizer.reset_results)
            if use_imu:
                rospy.Service('reset_imu_service', GetResults,
                            imu.reset)
        rospy.spin()


if __name__ == "__main__":
    try:
        node = A1SlamNode()
        node.launch_node()
    except rospy.ROSInterruptException:
        pass
