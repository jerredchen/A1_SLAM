"""
Implementation of ICP.
"""

import gtsam
import matplotlib.pyplot as plt
import numpy as np
from sklearn.neighbors import NearestNeighbors


def estimate_normals(scan, k_nearest):
    """Estimate the normal vectors of each point through
    Principle Component Analysis (PCA).
    Args:
        scan: A numpy array of shape (2,n) of the scan's points.
        k_nearest: The k-nearest neighbors to consider for estimating the normals.
    Returns:
        normals: A numpy array of shape (2,n) of normal vectors.
    """
    # Create a KD Tree to calculate the k-nearest neighbors of each point.
    neighbors = NearestNeighbors(n_neighbors=k_nearest).fit(scan.T)
    normals = np.zeros_like(scan, dtype=np.float64)
    for i, point in enumerate(scan.T):
        # Calculate the k-nearest neighboring points.
        inds = neighbors.kneighbors(
            point[np.newaxis], return_distance=False).flatten()
        neighborhood = scan[:, inds]

        # Calculate the eigenvalues/eigenvectors of the neighborhood covariance.
        centroid = np.mean(neighborhood, axis=1)
        covariance = np.cov(neighborhood - centroid[:, np.newaxis])
        eigenvalues, eigenvectors = np.linalg.eig(covariance)

        # Select the eigenvector corresponding to the smallest eigenvalue.
        if eigenvalues[0] < eigenvalues[1]:
            normal = eigenvectors[:, 0]
        else:
            normal = eigenvectors[:, 1]

        # Enforce normals to always point to center of the scan.
        normals[:, i] = normal if normal @ -point > 0 else -normal
    return normals


def transform_scan(scan, transform):
    """Transform a given point cloud using a specified transformation.
    Args:
        scan: A numpy array of shape (2, n) of the scan's points.
        transform: A gtsam.Pose2 object to transform the scan.
    Returns:
        transformed_scan: A (2, n) numpy array with transformed points.
    """
    return (transform.transformFrom(scan))


def align_nearest_points(scan_a, scan_b, normals):
    """Align the points and normals of scan_a with its nearest neighbor
    points in scan_b.
    Args:
        scan_a: A (2, n) numpy array of the scan in the previous time step.
        scan_b: A (2, m) numpy array of the scan in the current time step.
        normals: A (2, n) numpy array of the normals associated with scan_a.
                    Ignore if None.
    Returns:
    """
    # assert(scan_a.shape == normals.shape)
    neighbors = NearestNeighbors(n_neighbors=1, radius=0.05).fit(scan_a.T)
    ind = (neighbors.kneighbors(scan_b.T, return_distance=False).T)[0]
    scan_a_prime = ((scan_a.T)[ind]).T
    normals_prime = ((normals.T)[ind]).T if normals is not None else None
    return scan_a_prime, normals_prime


def estimate_transform(scan_a, scan_b, normals=None):
    """Estimate the transform between the two point clouds using SVD.
    Args:
        scan_a: A (2, n) numpy array of the scan in the previous time step.
        scan_b: A (2, n) numpy array of the scan in the current time step.
        normals: A (2, n) numpy array of the normals associated with scan_a.
                    If None, use point-to-point loss to estimate the transform.
    Returns:
        aTb: A gtsam.Pose2 relative pose between scan_a and scan_b.
    """
    if normals is not None:
        rotation_terms = np.cross(scan_b.T, normals.T)[:, np.newaxis]
        translation_terms = normals.T
        A = np.hstack((rotation_terms, translation_terms))
        b = (np.sum(-(scan_b.T - scan_a.T) * normals.T, axis=1))[:, np.newaxis]
        params = np.linalg.pinv(A.T @ A) @ A.T @ b
        aTb = gtsam.Pose2(params[1], params[2], params[0])
    else:
        aTb = gtsam.Pose2.Align(scan_a, scan_b)
    return aTb


def visualize_correspondences(scan_a, scan_b, time=0.5):
    """Visualize the correspondences made between two scans.
    Args:
        scan_a: A (2, n) numpy array of the scan in the previous time step.
        scan_b: A (2, n) numpy array of the scan in the current time step.
        time: The number of seconds to pause between each plot.
    """
    plt.clf()
    plt.scatter(scan_a[0], scan_a[1], s=1.0, c='g')
    plt.scatter(scan_b[0], scan_b[1], s=1.0, c='r')
    for k in range(len(scan_a[0])):
        x = [scan_a[0, k], scan_b[0, k]]
        y = [scan_a[1, k], scan_b[1, k]]
        plt.plot(x, y, linewidth=0.75, c='b')
    plt.pause(time)


def icp(scan_a,
        scan_b,
        init_aTb=gtsam.Pose2(),
        normals_a=None,
        max_iterations=25,
        plot_points=False):
    """Align two 2D LiDAR scans and estimate the relative pose aTb.
    Args:
        scan_a: A (2, n) numpy array of the scan in the previous time step.
        scan_b: A (2, m) numpy array of the scan in the current time step.
        init_aTb: A gtsam.Pose2 object of the initial ICP estimate.
        normals_a: The normal vectors associated with scan_a. If not None,
                    point-to-line loss is used instead of point-to-point.
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
        scan_a_prime, normals_a_prime = align_nearest_points(scan_a, scan_b, normals_a)

        # Plot the points and each associated correspondence if needed.
        if plot_points:
            visualize_correspondences(scan_a_prime, scan_b)

        # Estimate the transform between the aligned scan_b points and the transformed scan_a points.
        estimated_transform = estimate_transform(scan_a_prime, scan_b, normals_a_prime)

        # Check to see if the estimated transform changed from its previous iteration.
        if transform.equals(estimated_transform, tol=1e-2):
            break

        # Reinitialize the transforms for the next iteration.
        icp_transform = icp_transform.compose(estimated_transform)
        transform = estimated_transform

    return icp_transform
