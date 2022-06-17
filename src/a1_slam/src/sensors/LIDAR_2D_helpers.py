"""
Helper functions for setting up LIDAR odometry in the PoseSLAM factor graph.
"""
import rospy

import gtsam
import numpy as np
from gtsam.symbol_shorthand import X
from registration import icp_line, vanilla_ICP
from sensor_msgs.msg import LaserScan


def _parse_config_parameters(prior_pose_estimate,
                             prior_pose_sigmas,
                             icp_noise_sigmas):
    """Parse the config parameters to be used as gtsam objects.

    Args:
        prior_pose_estimate:    A list in format [x, y, theta]
                                    representing the initial estimate on the prior pose.
        prior_pose_sigmas:      A list in format [x, y, theta] representing the prior
                                    pose noise's translational and rotational standard deviation.
        icp_noise_sigmas:       A list in format [x, y, theta] representing the noise
                                    standard deviations associated with a LIDAR odometry factor.
    Returns:
        prior_pose_factor:      The prior factor associated with the initial pose.
        icp_noise_model:        A noise model representing the noise from LIDAR registration.
    """
    pose = gtsam.Pose2(
        prior_pose_estimate[0],
        prior_pose_estimate[1],
        np.deg2rad(prior_pose_estimate[2])
    )
    prior_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([
            prior_pose_sigmas[0],
            prior_pose_sigmas[1],
            np.deg2rad(prior_pose_sigmas[2])
        ])
    )
    prior_pose_factor = gtsam.PriorFactorPose2(
        X(0), pose, prior_noise_model
    )
    icp_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([
            icp_noise_sigmas[0],
            icp_noise_sigmas[1],
            np.deg2rad(icp_noise_sigmas[2])
        ])
    )
    return prior_pose_factor, icp_noise_model


def _create_factor_graph_and_params(
        graph: gtsam.NonlinearFactorGraph,
        initial_estimates: gtsam.Values(),
        prior_pose_factor: gtsam.PriorFactorPose2):
    """Create a factor graph and the necessary parameters for optimization.

    Args:
        prior_pose_factor:  The prior factor associated with the initial pose.
    Returns:
        graph:              A gtsam.NonlinearFactorGraph with the priors inserted.
        initial_estimates:   A gtsam.Values with the initial estimates stored.
    """

    # Add the prior factors to the factor graph, and initialize the estimates.
    graph.add(prior_pose_factor)
    initial_estimates.insert(X(0), prior_pose_factor.prior())

    return graph, initial_estimates


def create_lidar_graph_and_params(graph, initial_estimates):
    """Parses the rosparams to setup the relevant LIDAR graph and parameters.

    Args:
        graph: The initialized factor graph.
        initial_estimates: The initialized initial estimates.
        icp_noise_model: The noise model from performing ICP registration.
    """

    # Obtain the poses estimate, in meters and degrees.
    prior_pose_estimate = rospy.get_param('prior_pose_estimate')

    # Obtain the pose's translational and rotational standard deviations, in meters and degrees.
    prior_pose_sigmas = rospy.get_param('prior_pose_sigmas')

    # Obtain the standard deviation associated with registration noise, in meters and degrees.
    icp_noise_sigmas = rospy.get_param('icp_noise')

    # Parse the config parameters to be GTSAM appropriate objects.
    prior_pose_factor, icp_noise_model = _parse_config_parameters(
        prior_pose_estimate, prior_pose_sigmas, icp_noise_sigmas)

    # Create the initial factor graph and associated parameters for setup.
    graph, initial_estimates = _create_factor_graph_and_params(
        graph, initial_estimates, prior_pose_factor)

    return graph, initial_estimates, icp_noise_model


def preprocess_measurement(laser_msg: LaserScan):
    """Extracts ranges from the LaserScan message and converts into
    a 3*N homogenous matrix of points.
    Args:
        laser_msg: The LaserScan message from the A1.
    Returns:
        scan: A numpy array of shape (3, N), where N is the number of 2D points.
    """
    scan = np.zeros((2, len(laser_msg.ranges)), dtype=float)
    # Compute the 2D point where the angle starts at +90 deg, and
    # each range is spaced out in equal increments.
    for i, distance in enumerate(laser_msg.ranges):
        if 0 < distance < 5:
            scan[0][i] = distance * \
                np.cos(laser_msg.angle_min + i*laser_msg.angle_increment)
            scan[1][i] = distance * \
                np.sin(laser_msg.angle_min + i*laser_msg.angle_increment)
    # Remove all values in the array that are still 0.
    mask = scan[0] != 0
    scan = scan[:, mask]
    # Append ones to make points homogenous.
    scan = np.vstack((scan, np.ones((1, len(scan[0])))))
    return scan


def add_lidar_factor(a: int,
                     b: int,
                     scan_a: np.ndarray,
                     scan_b: np.ndarray,
                     icp_noise: gtsam.noiseModel,
                     graph: gtsam.NonlinearFactorGraph,
                     initial_estimates: gtsam.Values,
                     results: gtsam.Values,
                     registration="point-to-line"):
    """Adds an LIDAR odometry factor to the factor graph and
    relevant initial estimates.
    Args:
        a: The previous state index of the odometry factor.
        b: The subsequent state index of the odometry factor.
        scan_a: The LIDAR scan associated with state a.
        scan_b: The LIDAR scan associated with state b.
        icp_noise: A gtsam noise model of the LIDAR registration noise.
        graph: The current factor graph before adding the odometry factor.
        initial_estimates: The current set of initial estimates.
        results: The current set of pose estimates for the trajectory.
        registration: Specified ICP registration, default to point-to-line.
    Returns:
        graph: An updated factor graph with the odometry factor.
        initial_estimates: Updates initial estimates.
    """
    if results.exists(X(b)):
        wTa, wTb = results.atPose2(X(a)), results.atPose2(X(b))
        initial_transform = wTa.between(wTb)
    elif b > 1:
        wTa, wTb = results.atPose2(X(b-2)), results.atPose2(X(b-1))
        initial_transform = wTa.between(wTb)
        initial_estimates.insert(X(b), wTb.compose(initial_transform))
    else:
        initial_transform = gtsam.Pose2()
        initial_estimates.insert(X(b), results.atPose2(X(0)))
    if registration == "point-to-line":
        aTb = icp_line.icp(scan_b, scan_a, initial_transform)
    elif registration == "vanilla":
        aTb = vanilla_ICP.icp(scan_b, scan_a, initial_transform)
    graph.add(gtsam.BetweenFactorPose2(X(a), X(b), aTb, icp_noise))
    return graph, initial_estimates
