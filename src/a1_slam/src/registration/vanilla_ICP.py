"""
Personal implementation of standard vanilla ICP.
"""

import matplotlib.pyplot as plt
import numpy as np
from sklearn.neighbors import NearestNeighbors

import gtsam

def transform_scan(scan_a, transform):
    """Transform a given point cloud using a specified transformation."""
    return (transform.matrix() @ scan_a)

def align_nearest_points(scan_a, scan_b):
    """Align the points in scan_a with its nearest neighbor points in scan_b."""
    scan_bT = scan_b[:].T
    neighbors = NearestNeighbors(n_neighbors=1, radius=0.05).fit(scan_bT)
    ind = (neighbors.kneighbors(scan_a.T, return_distance=False).T)[0]
    return (scan_bT[ind]).T

def estimate_transform_SVD(scan_a, scan_b):
    """Estimate the transform between the two point clouds using SVD."""
    centroid_a = np.expand_dims(np.average(scan_a, axis=1), 1)
    centroid_b = np.expand_dims(np.average(scan_b, axis=1), 1)
    scan_a_prime = scan_a - centroid_a
    scan_b_prime = scan_b - centroid_b
    H = np.sum(scan_a_prime.T[:,:,None]*scan_b_prime.T[:,None], axis=0)
    u, _, vh = np.linalg.svd(H)
    R_mat = vh.T @ u.T
    if np.isclose(np.linalg.det(R_mat), -1):
        v = vh.T
        R_mat = np.hstack((v[:,0,None], v[:,1,None], -v[:,2,None]))
    theta = np.arctan2(R_mat[1,0], R_mat[0,0])
    t_vec = centroid_b - (R_mat @ centroid_a)
    return gtsam.Pose2(theta, t_vec[:2])

def icp(scan_a, scan_b, initial_transform=gtsam.Pose2(), max_iterations=25, plotPoints=False):
    """Perform standard vanilla ICP between two 2D LiDAR scans."""
    delta = float('inf')
    threshold = 0.01
    transform = initial_transform
    icp_transform = initial_transform

    i = 0
    while delta > threshold and i < max_iterations:
        # Transform the scan using the previously used transform.
        scan_a = transform_scan(scan_a, transform)

        # Align the nearest neighbor points in scan_b to scan_a.
        scan_c = align_nearest_points(scan_a, scan_b)

        # Plot the points and each associated correspondence if needed.
        if plotPoints:
            plt.clf()
            plt.scatter(scan_a[0], scan_a[1], s=1.0, c='g')
            plt.scatter(scan_b[0], scan_b[1], s=1.0, c='r')
            for k in range(len(scan_a[0])):
                x = [scan_a[0,k], scan_c[0,k]]
                y = [scan_a[1,k], scan_c[1,k]]
                plt.plot(x, y, linewidth=0.75, c='b')
            plt.pause(0.5)

        # Estimate the transform between the aligned scan_b points and the transformed scan_a points.
        estimated_transform = estimate_transform_SVD(scan_a, scan_c)

        # Check to see if the estimated transform changed from its previous iteration.
        delta = np.linalg.norm(scan_c - scan_a)

        # Reinitialize the transforms for the next iteration.
        icp_transform = icp_transform.compose(estimated_transform)
        transform = estimated_transform
        i += 1

    return icp_transform
