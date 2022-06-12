"""
Personal implementation of standard vanilla ICP.
"""

import matplotlib.pyplot as plt
import numpy as np
from sklearn.neighbors import NearestNeighbors
from scipy.stats import multivariate_normal

import gtsam
import math 
#import gpc

def transform_scan(scan_a, transform):
    """Transform a given point cloud using a specified transformation."""
    return (transform.matrix() @ scan_a)

## Return the transform and the loss
def transform_point_to_plane(scan_a, scan_b, transform, num_points = 2, radius=0.5):

    ## Making the size of the point list the same in both the scans
    size = min(len(scan_b[0]), len(scan_a[0]))
    
    scan_a = scan_a[:,:size]
    scan_b = scan_b[:,:size]
    
    ## Get the K nearest neigbors
    neighbors = NearestNeighbors(n_neighbors=num_points).fit(scan_b.T)
    #ind = (neighbors.kneighbors(scan_a.T, return_distance=False).T)

    dist, ind = np.asarray(neighbors.kneighbors(scan_a.T, return_distance=True))
    
    loss = 0
    loss_point_sum = 0
    scan_a_f = []
    scan_c = []
    fil_points = size
    H00 = np.zeros((2,2))
    H01 = np.zeros((2,1))
    H10 = np.zeros((1,2))
    H11 = np.zeros((1,1))
    for i in range(size):
        ## Get the k nearest points
        points = []
        for j in range(ind.shape[1]):
            d = dist[i]
            if(d[j] <= radius):
                points.append(scan_b.T[int(ind[i,j])])
        
        points = np.asarray(points)
        #print(i, points, points.shape)
        
        if(points.shape[0] > 0 and transform == 'point'):
            ## Append the new point and the point to the scans
            scan_c.append(points[0])
            scan_a_f.append(scan_a.T[i])
            fil_points -= 1
        elif(points.shape[0] > 1 and transform == 'line'):
            ## Compute the centroid of the k nearest neighbors
            centroid_point = np.mean(points, axis=0)

            # Compute the Covariance
            covar = np.zeros((3,3))
            for j in range(points.shape[0]):
                diff = (points[j] - centroid_point).reshape(3,1)
                mult = np.dot(diff, diff.T)
                covar += mult
            covar = covar/points.shape[0]

            ## Eigen value and Eigen Vector
            eigen_val, eigen_vec = np.linalg.eig(covar)
        
            ## Use the smallest eigen value corresponding eigen vector as the normal
            ev_s = eigen_vec[:,0].T if (eigen_val[0] < eigen_val[1]) else eigen_vec[:,1].T
        
            nx, ny = ev_s[0], ev_s[1]
            dx, dy = centroid_point[0], centroid_point[1]
            sx, sy = scan_a.T[i][0], scan_a.T[i][1]
    
            p = np.asarray([sx, sy]).reshape(2,1)
            n = np.asarray([nx, ny]).reshape(1,2)
            
            pn = np.multiply(p, n)
            H00 += np.dot(pn, pn.T)
            H01 += np.dot(pn, n.T)
            H10 += np.dot(n, pn)
            H11 += np.dot(n, n.T)

            #print("D:", i, p.shape, n.shape, pn.shape, H00.shape, H01.shape, H10.shape, H11.shape)
            ## Compute m1 and m2
            if (nx == 0):
                xn = sx
                yn = dy
            elif (ny == 0):
                xn = dx
                yn = sy
            else:
                m1 = ny/nx
                m2 = -nx/ny  # -1/m1
                ## Compute b1 and b2
                b1 = sy - (m1*sx)
                b2 = dy - (m2*dx)

                ## compute the new points on the normal (xn,yn)
                xn = np.divide((b2-b1), (m1-m2))
                yn = m2 * xn + b2
            
            ## The new points on the normal
            tr = [xn,yn,1.0]
              
            """
            if True:
                plt.scatter(sx, sy, s=1.0, c='g')
                #plt.scatter(nx, ny, s=1.0, c='r')
                plt.scatter(dx, dy, s=1.0, c='y')
                plt.scatter(xn, yn, s=1.0, c='k')
            
                x = [sx, xn]
                y = [sy, yn]
                n = [sx, dx]
                m = [sy, dy]
                f = [dx, xn]
                s = [dy, yn]
                plt.plot(x, y, linewidth=0.75, c='b')
                plt.plot(n, m, linewidth=1.5, c='c')
                plt.plot(f, s, linewidth=0.75, c='m')
            """
        
            ## Loss function
            loss_point = np.mean(np.power(np.dot((scan_a.T[i] - centroid_point), ev_s), 2))
            loss_point_sum += loss_point
            
            ## Append the new point and the point to the scans
            scan_c.append(tr)
            scan_a_f.append(scan_a.T[i])
            fil_points -= 1
            
    #plt.pause(5)
    #print("H00:", H00.shape, H00, H00[0][0], H00[0][1], H00[1][0], H00[1][1])
    #print("H01:", H01.shape, H01, H01[0][0], H01[1][0])
    #print("H10:", H10.shape, H10, H10[0][0], H10[0][1])
    #print("H11:", H11.shape, H11, H11[0][0])
    
    variance = 0.00027777777
    """
    covarianceMatrix = np.array([[H00[0][0], H00[0][1], H01[0][0]], [H00[1][0], H00[1][1], H01[1][0]], [H10[0][0], H10[0][1], H11[0][0]]], dtype = 'float64')
    #print("cov:", covarianceMatrix, covarianceMatrix.shape)
    covarianceMatrixInv = np.linalg.inv(covarianceMatrix)
    covarianceMatrix = np.multiply(variance, covarianceMatrixInv)
    print(covarianceMatrix)
    """
    scan_c = np.asarray(scan_c).T
    scan_a_f = np.asarray(scan_a_f).T
    #print("Scan_A:", scan_a_f.shape)
    #print("Scan_C:", scan_c.shape)
    #print("Total points:", size, "Filtered Points:", fil_points)
    delta = np.linalg.norm(scan_c - scan_a_f)
    
    if(points.shape[0] > 0 and transform == 'point'):
        loss_point_sum = delta
    
    if(size == fil_points):
        print("All the points in the scan are filtered, change the radius")
        return transform, scan_c, loss_point_sum, delta, 1
		

    """Estimate the transform between the two point clouds using SVD."""
    centroid_a = np.expand_dims(np.average(scan_a_f, axis=1), 1)
    centroid_c = np.expand_dims(np.average(scan_c, axis=1), 1)
    scan_a_prime = scan_a_f - centroid_a
    scan_c_prime = scan_c - centroid_c
    H = np.sum(scan_a_prime.T[:,:,None]*scan_c_prime.T[:,None], axis=0)
    u, _, vh = np.linalg.svd(H)
    R_mat = vh.T @ u.T
    if np.isclose(np.linalg.det(R_mat), -1):
        v = vh.T
        R_mat = np.hstack((v[:,0,None], v[:,1,None], -v[:,2,None]))
    theta = np.arctan2(R_mat[1,0], R_mat[0,0])
    t_vec = centroid_c - (R_mat @ centroid_a)
    return gtsam.Pose2(theta, t_vec[:2]), scan_c, loss_point_sum, delta, 0

def icp(scan_a, scan_b, initial_transform=gtsam.Pose2(), max_iterations=50, plotPoints=False):
    """Perform standard vanilla ICP between two 2D LiDAR scans."""
    #delta = float('inf')
    delta_knn = float('inf')
    threshold = 0.01
    transform = initial_transform
    icp_transform = initial_transform
    skip = 0

    i = 0
    while delta_knn > threshold and i < max_iterations:
        # Transform the scan using the previously used transform.
        scan_a = transform_scan(scan_a, transform)

        # Estimate the transform between scan_a(source) and scan_b(target) and the loss
        estimated_transform, scan_c, delta_knn, delta, skip = transform_point_to_plane(scan_a, scan_b, transform = 'line')
        if skip: break

        # Plot the points and each associated correspondence if needed.
        if plotPoints:
            plt.clf()
            plt.scatter(scan_a[0], scan_a[1], s=1.0, c='g')
            plt.scatter(scan_b[0], scan_b[1], s=1.0, c='r')
            for k in range(min(len(scan_a[0]), len(scan_b[0]))):
                x = [scan_a[0,k], scan_b[0,k]]
                y = [scan_a[1,k], scan_b[1,k]]
                plt.plot(x, y, linewidth=0.75, c='b')
            plt.pause(0.5)
        
        # Check to see if the estimated transform changed from its previous iteration.
        #delta = np.linalg.norm(scan_c - scan_a)
        #print("Interation:", i, delta_knn, delta, threshold)
        # Reinitialize the transforms for the next iteration.
        icp_transform = icp_transform.compose(estimated_transform)
        transform = estimated_transform
        i += 1

    return icp_transform, scan_c, skip
