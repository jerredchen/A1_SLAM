#!/usr/bin/env python

import gtsam
import numpy as np
from nodes.Lidar2DNode import Lidar2D
import rospy
import threading
from a1_slam.srv import GetResults, ClearResults
from collections import deque
from geometry_msgs.msg import PoseStamped
from gtsam.symbol_shorthand import B, V, X
from nav_msgs.msg import Path
from threading import Lock

class A1SlamNode():

    # Subscriber callbacks

    def lidar_callback(self, msg):
        """
        Trying to isolate functions for better package structure
        (as opposed to having all functions shoved into a single file).
        Pros:
            - better readibility
            - better structure of package
            - more modular
        Difficulties:
            - can you have a callback in a separate module?
                - you need a way to place the factor in the queue
                - one way to circumvent is have queue as attribute of object,
                    accept the object as arg for callback
                    e.g. lidar_callback(self, msg, optimizer) where
                    optimizer.queue is an attribute
                    - how do have optimizer callbacks work (continuously optimize)?
                        - 
        """
        factor = Lidar2D.create_lidar_factor()
        self.factor_queue.appendleft(factor)
    
    def imu_callback(self, msg):
        pass

    def launch_node(self):
        """Initialize and start the A1 SLAM node."""
        rospy.init_node('a1_slam_node', anonymous=True)

        # Create a prior pose factor and add to the factor graph.
        prior_pose_factor = self.create_prior_pose_factor(
            rospy.get_param('/prior_pose_estimate'),
            rospy.get_param('/prior_pose_sigmas')
        )
        self.graph.add(prior_pose_factor)
        self.init_estimates.insert(
            X(0),
            prior_pose_factor.prior()
        )

        # Perform an iSAM2 incremental update.
        self.isam.update(self.graph, self.initial_estimates)
        self.results = self.isam.calculateEstimate()

        # Clear the graph and initial estimates.
        self.graph = gtsam.NonlinearFactorGraph()
        self.init_estimates.clear()

        # Create a Lidar2D wrapper that subscribes to laser scans.
        sub = rospy.Subscriber()

        # Create an Imu wrapper that subscribes to imu.

        # Create an optimizer that pops from the queues and optimizes the factor graph.
        rospy.Timer(0.1, )

        # Create a pose publisher that publishes the pose from the results.
        rospy.Timer(0.5, )

        # Create a trajectory publisher that publishes the trajectory of results.
        rospy.Timer(1.0, )

        # Start the necessary services for sending results.
        rospy.Service('get_results_service', GetResults,
                      self.send_results_callback)
        # Clear results service is only used for testing purposes.
        rospy.Service('clear_results_service', ClearResults,
                      self.clear_results_callback)

        rospy.spin()

if __name__ == "__main__":
    try:
        node = A1SlamNode()
        node.launch_node()
    except rospy.ROSInterruptException:
        pass
