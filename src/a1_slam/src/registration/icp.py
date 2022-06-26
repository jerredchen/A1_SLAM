"""
Implementation of ICP.
"""

import gtsam
import matplotlib.pyplot as plt
import numpy as np
from sklearn.neighbors import NearestNeighbors


def estimate_normals(scan):
    """Estimate the normal vectors of each point through
    Principle Component Analysis (PCA).
    Args:
        scan: A numpy array of shape (2,n) of the scan's points.
    Returns:
        normals: A numpy array of shape (2,n) of normal vectors.
    """
    pass


def transform_scan(scan, transform):
    """Transform a given point cloud using a specified transformation.
    Args:
        scan: A numpy array of shape (2, n) of the scan's points.
        transform: A gtsam.Pose2 object to transform the scan.
    Returns:
        transformed_scan: A (2, n) numpy array with transformed points.
    """
    return (transform.transformFrom(scan))


def align_nearest_points(scan_a, scan_b):
    """Align the points in scan_a with its nearest neighbor points in scan_b.
    """
    scan_bT = scan_b[:].T
    neighbors = NearestNeighbors(n_neighbors=1, radius=0.05).fit(scan_bT)
    ind = (neighbors.kneighbors(scan_a.T, return_distance=False).T)[0]
    return ((scan_b.T)[ind]).T


def estimate_transform(scan_a, scan_b, point_to_line=True):
    """Estimate the transform between the two point clouds using SVD.
    Args:
        scan_a: A (2, n) numpy array of the source scan.
        scan_b: A (2, n) numpy array of the target scan.
        point_to_line: Whether to use the point to line loss.
    Returns:
        aTb: A gtsam.Pose2 relative pose between scan_a and scan_b.
    """
    return gtsam.Pose2.Align(scan_a, scan_b)


def icp(scan_a, scan_b, init_aTb=gtsam.Pose2(), point_to_line=True, max_iterations=25, plot_points=False):
    """Align two 2D LiDAR scans and estimate the relative pose aTb.
    Args:
        scan_a: A (2, n) numpy array of the source scan.
        scan_b: A (2, m) numpy array of the target scan.
        init_aTb: A gtsam.Pose2 object of the initial ICP estimate.
        point_to_line: Whether to use point to line loss.
        max_iterations: The maximum number of iterations to perform ICP.
        plot_points: Whether to visualize the point correspondences.
    Returns:
        aTb: A gtsam.Pose2 of the relative pose between scan a and b.
    """
    transform = init_aTb
    icp_transform = init_aTb

    for _ in range(max_iterations):
        # Transform the scan using the previously used transform.
        scan_b = transform_scan(scan_b, transform)

        # Align the nearest neighbor points in scan_a to scan_b.
        scan_c = align_nearest_points(scan_b, scan_a)

        # Plot the points and each associated correspondence if needed.
        if plot_points:
            plt.clf()
            plt.scatter(scan_a[0], scan_a[1], s=1.0, c='g')
            plt.scatter(scan_b[0], scan_b[1], s=1.0, c='r')
            for k in range(len(scan_a[0])):
                x = [scan_a[0, k], scan_c[0, k]]
                y = [scan_a[1, k], scan_c[1, k]]
                plt.plot(x, y, linewidth=0.75, c='b')
            plt.pause(0.5)

        # Estimate the transform between the aligned scan_b points and the transformed scan_a points.
        estimated_transform = estimate_transform(scan_c, scan_b)

        # Check to see if the estimated transform changed from its previous iteration.
        if transform.equals(estimated_transform, tol=1e-2):
            break

        # Reinitialize the transforms for the next iteration.
        icp_transform = icp_transform.compose(estimated_transform)
        transform = estimated_transform

    return icp_transform
