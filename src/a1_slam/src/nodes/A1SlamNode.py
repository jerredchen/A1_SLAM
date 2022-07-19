#!/usr/bin/env python

import rospy
from a1_slam.srv import GetResults
from sensor_msgs.msg import Imu, LaserScan
from unitree_legged_msgs.msg import HighState

from optimization.optimizer import Optimizer
from sensors.imu import ImuWrapper
from sensors.lidar2D import Lidar2DWrapper
class A1SlamNode():

    def launch_node(self):
        """Initialize and start the A1 SLAM node."""
        rospy.init_node('a1_slam_node', anonymous=True)

        # Check which sensors are selected to be used for SLAM.
        use_imu = rospy.get_param('/use_imu')
        use_2dlidar = rospy.get_param('/use_2dlidar')
        use_depth = rospy.get_param('/use_depth')
        assert any([use_imu, use_2dlidar, use_depth]), "No sensors were selected."

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
            lidar = Lidar2DWrapper(optimizer)
            lidar.initialize_params()
            lidar_topic = rospy.get_param('/lidar2d/topic')
            rospy.Subscriber(
                lidar_topic,
                LaserScan,
                lidar.lidar_callback, 
                callback_args=imu
            )

        optimizer.optimize()

        # Create a pose publisher that publishes the pose from the results.
        rospy.Timer(rospy.Duration(0.1), optimizer.pose_callback)

        # Create a trajectory publisher that publishes the trajectory of results.
        rospy.Timer(rospy.Duration(1.0), optimizer.trajectory_callback)

        # Start the necessary services for sending results.
        rospy.Service('get_results_service', GetResults,
                      optimizer.send_results)
        # Reset results service is only used for testing purposes.
        rospy.Service('reset_results_service', GetResults,
                      optimizer.reset_results)
        rospy.spin()


if __name__ == "__main__":
    try:
        node = A1SlamNode()
        node.launch_node()
    except rospy.ROSInterruptException:
        pass
