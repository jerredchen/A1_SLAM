#!/usr/bin/env python

"""
Node which optimizes the factor graph using iSAM2.
Current design:
- Separate service to accept a specific sensor. For each service,
    the optimized results are returned to the respective sensor.
- Each sensor is its own separate process node. This allows for:
    - easy testing. Test cases only depend on launching a specific node.
        - Even if they weren't separate nodes, would that still work?
            The original issue was that you could not figure out how to
            test the nodes individually. However, currently all nodes are
            to be launched and only specific streams of data are published.
    - each sensor now runs on a separate process and can perform processing
        simultaneously instead of each sensor running on a separate thread
        in the same process.
Possible redesigns for optimizer node:
- Each node publishes a factor and optimizer has several subscribers.
    This could be used if there is not a need to immediately receive
    the results publishing the factor.
    - The results would then be obtain through a continuous publisher.
- There is a shared queue (with proper mutexes) between nodes where
    the factors are added to the queue and the optimizer pops the factors
    from the added queue continuously. The optimizer continuously optimizes
    factors from the queue and the results are then updated. Results are also
    accessed from a shared location of memory.
- There is only one node which is the a1_slam node, and the node consists of
    multiple subscribers to the sensor topics.

"""
import gtsam
import numpy as np
import rospy
from a1_slam.msg import (BetweenFactorPose3, ImuFactor,
                         PriorFactorConstantBias, PriorFactorPose3,
                         PriorFactorVector)
from a1_slam.srv import AddFactor, ClearResults, GetResults
from geometry_msgs.msg import PoseStamped
from gtsam.symbol_shorthand import B, V, X
from nav_msgs.msg import Path


class OptimizerNode():
    def __init__(self):

        # Instantiate publisher attributes.
        self.pose_publisher = rospy.Publisher(
            "/pose_estimate", PoseStamped, queue_size=5
        )
        self.traj_publisher = rospy.Publisher(
            "/traj_estimate", Path, queue_size=5
        )

        # Instantiate factor graph and optimizer attributes.
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()
        self.results = gtsam.Values()
        self.isam = gtsam.ISAM2(gtsam.ISAM2Params())

    ############ Parsing parameters ############

    def create_prior_pose_factor(self,
                                 prior_pose_estimate,
                                 prior_pose_sigmas):
        """Parse the parameters and return a PriorFactorPose3.
        Args:
            prior_pose_estimate:    A list in format [x, y, z, roll, pitch, yaw]
                                        representing the initial estimate on the prior pose.
            prior_pose_sigmas:      A list in format [x, y, z, roll, pitch, yaw] representing the prior
                                        pose noise's translational and rotational standard deviations.
        """
        rpy = np.deg2rad(prior_pose_estimate[3:])
        pose_rotation = gtsam.Rot3.Ypr(*(rpy[::-1]))
        pose_translation = gtsam.Point3(*prior_pose_estimate[:3])
        prior_pose_estimate = gtsam.Pose3(pose_rotation, pose_translation)
        prior_pose_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.concatenate(
                (np.deg2rad(prior_pose_sigmas[3:]), prior_pose_sigmas[:3]))
        )
        prior_pose_factor = gtsam.PriorFactorPose3(
            X(0), prior_pose_estimate, prior_pose_noise)
        return prior_pose_factor

    ############ Service callbacks ############

    def send_results_callback(self, request):
        serialized_str = self.results.serialize()
        return serialized_str

    def clear_results_callback(self, request):
        serialized_str = self.results.serialize()
        self.results.clear()
        rospy.logwarn("Results cleared")
        return serialized_str

    def BetweenFactorPose3_callback(self, request):
        pose_trans = np.array(request.xyz)
        pose_rot = gtsam.Rot3.Ypr(*(request.rpy[::-1]))
        relative_pose = gtsam.Pose3(pose_rot, pose_trans)
        noise_model = gtsam.noiseModel.Diagonal.Sigmas(
            np.array(request.sigmas))
        factor = gtsam.BetweenFactorPose3(
            request.key_i,
            request.key_j,
            relative_pose,
            noise_model
        )

    def ImuFactor_callback(self, request):
        bias = gtsam.imuBias.ConstantBias(
            np.array(request.accel_bias),
            np.array(request.gyro_bias)
        )
        pim_params = gtsam.PreintegrationParams.MakeSharedU()
        pim = gtsam.PreintegratedImuMeasurements(pim_params)
        pim.deserialize(request.serialized_pim)
        factor = gtsam.ImuFactor(
            request.pose_key_i,
            request.vel_key_i,
            request.pose_key_j,
            request.vel_key_j,
            request.bias_key,
            bias,
            pim
        )

    def PriorFactorConstantBias_callback(self, request):
        bias = gtsam.imuBias.ConstantBias(
            np.array(request.accel_bias),
            np.array(request.gyro_bias)
        )
        noise_model = gtsam.noiseModel.Diagonal.Sigmas(
            np.array(request.sigmas))
        factor = gtsam.PriorFactorConstantBias(
            request.key,
            bias,
            noise_model
        )

    def PriorFactorPose3_callback(self, request):
        pose_trans = np.array(request.xyz)
        pose_rot = gtsam.Rot3.Ypr(*(request.rpy[::-1]))
        pose = gtsam.Pose3(pose_rot, pose_trans)
        noise_model = gtsam.noiseModel.Diagonal.Sigmas(
            np.array(request.sigmas))
        factor = gtsam.PriorFactorConstantBias(
            request.key,
            pose,
            noise_model
        )

    def PriorFactorVector_callback(self, request):
        noise_model = gtsam.noiseModel.Diagonal.Sigmas(
            np.array(request.sigmas))
        factor = gtsam.PriorFactorConstantBias(
            request.key,
            np.array(request.vector),
            noise_model
        )

    def optimize_graph(self, factor, initial_estimates):
        """Optimize the factor graph using iSAM2.
        Args:
            factor: The factor to be added to the factor graph.
            initial_estimates: A list of tuples in the format of
                                (key, initial_estimate).
        """
        new_pose_key = -1
        self.graph.add(factor)
        for key, estimate in initial_estimates:
            if not self.results.exists(key):
                self.initial_estimates.insert(key, estimate)
                new_pose_key = key if type(estimate) == gtsam.Pose3 else -1

        # Perform an iSAM2 incremental update.
        self.isam.update(self.graph, self.initial_estimates)
        self.results = self.isam.calculateEstimate()

        # Publish pose if new pose was added to trajectory.
        if new_pose_key != -1:
            # pose = self.results.atPose2()
            # rospy.loginfo(f"")
            pose_msg = self.process_pose_message(self.results.atPose3(new_pose_key))
            self.publish_pose(pose_msg)

        # Clear the graph and initial estimates.
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates.clear()

        # Serialize the results to return.
        serialized_str = self.results.serialize()
        return serialized_str

    def create_trajectory_callback(self, event=None):
        trajectory = []
        poses = gtsam.utilities.allPose3s(self.results)
        keys = gtsam.KeyVector(poses.keys())
        for key in keys:
            try:
                pose_msg = self.process_pose_message(key)
                trajectory.append(pose_msg)
            except:
                rospy.logwarn("Could not access key in results")
        self.publish_traj(trajectory)

    ############ Processing messages and publisher functions ############

    def process_pose_message(self, key):
        """Publish a PoseStamped message of the most recent pose."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"
        pose_estimate = self.results.atPose3(key)
        pose_msg.pose.position.x = pose_estimate.x()
        pose_msg.pose.position.y = pose_estimate.y()
        pose_msg.pose.position.y = pose_estimate.z()
        quaternion = pose_estimate.rotation().quaternion()
        pose_msg.pose.orientation.w = quaternion[0]
        pose_msg.pose.orientation.x = quaternion[1]
        pose_msg.pose.orientation.y = quaternion[2]
        pose_msg.pose.orientation.z = quaternion[3]
        return pose_msg

    def publish_pose(self, pose_msg):
        self.pose_publisher.publish(pose_msg)

    def publish_traj(self, trajectory):
        """Publish a Path message of the robot's trajectory."""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "world"
        path_msg.poses = trajectory
        self.traj_publisher.publish(path_msg)

    ############ Launch main optimizer node ############

    def launch_optimizer_node(self):
        """Initialize and start the optimizer node."""
        rospy.init_node('optimizer_node', anonymous=True)

        # Start a thread for publishing the trajectory._and_clear_
        rospy.Timer(rospy.Duration(0.5), self.create_trajectory_callback)

        # Instantiate the iSAM2 parameters to create the iSAM2 object.
        parameters = gtsam.ISAM2Params()
        self.isam = gtsam.ISAM2(parameters)

        # Create a prior pose factor and add to the factor graph.
        prior_pose_factor = self.create_prior_pose_factor(
            rospy.get_param('/prior_pose_estimate'),
            rospy.get_param('/prior_pose_sigmas')
        )
        self.graph.add(prior_pose_factor)
        self.initial_estimates.insert(
            X(0),
            prior_pose_factor.prior()
        )

        # Perform an iSAM2 incremental update.
        self.isam.update(self.graph, self.initial_estimates)
        self.results = self.isam.calculateEstimate()

        # Clear the graph and initial estimates.
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates.clear()

        # Start the necessary services for sending results.
        rospy.Service('get_results_service', GetResults,
                      self.send_results_callback)
        # Clear results service is only used for testing purposes.
        rospy.Service('clear_results_service', ClearResults,
                      self.clear_results_callback)

        # Start the necessary services for each factor.
        rospy.Service(
            'BetweenFactorPose3_service',
            BetweenFactorPose3,
            self.BetweenFactorPose3_callback
        )
        rospy.Service(
            'ImuFactor_service',
            ImuFactor,
            self.ImuFactor_callback
        )
        rospy.Service(
            'PriorFactorConstantBias_service',
            PriorFactorConstantBias,
            self.PriorFactorConstantBias_callback
        )
        rospy.Service(
            'PriorFactorPose3_service',
            PriorFactorPose3,
            self.PriorFactorPose3_callback
        )
        rospy.Service(
            'PriorFactorVector_service',
            PriorFactorVector,
            self.PriorFactorVector_callback
        )

        rospy.spin()


if __name__ == "__main__":
    try:
        node = OptimizerNode()
        node.launch_optimizer_node()
    except rospy.ROSInterruptException:
        pass
