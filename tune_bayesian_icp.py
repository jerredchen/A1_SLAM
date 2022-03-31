import matplotlib.pyplot as plt
import numpy as np
import rosbag

import gtsam

import A1_LiDAR_ISAM2
import SLAM.vanilla_ICP as vanilla_ICP
import sgdicp

def tune_bayesian_icp(bag_name: str,
                      topic_name: str,
                      scan_number: int):
    # Open the ROS bag to be used.
    bag = rosbag.Bag(bag_name)
    scan_transform = gtsam.Pose2()

    # Perform dead reckoning with vanilla ICP before reaching desired scan number
    # to use previous transform as an initial estimate and prior for Bayesian ICP.
    for k, bag_info in enumerate(bag.read_messages(topic_name)):

        # Extract the LiDAR message from the bag at a particular time instance.
        msg = bag_info[1]

        # Initialize scans from the previous iteration.
        if k > 0:
            scan_prev = scan

        # Convert the scan from a 1D array of ranges to an array of 2D points in the local pose frame.
        scan = A1_LiDAR_ISAM2.ranges_to_points(msg.ranges)

        if 0 < k < scan_number:
            # Estimate the transform between two consecutive scan measurements.
            scan_transform = vanilla_ICP.icp(scan, scan_prev, scan_transform)
        
        # Perform Bayesian ICP and plot result.
        elif k == scan_number:

            x = scan_transform.x()
            y = scan_transform.y()
            theta = scan_transform.theta()
            samples = sgdicp.infer_bayesian_icp_posterior(scan, scan_prev, [x, y, theta], [0,0,0], 0.125)

            burnin = 100
            posterior = samples[burnin:, :]
            mean_posterior = np.mean(posterior, axis=0)
            actual = gtsam.Pose2(mean_posterior[0], mean_posterior[1], mean_posterior[2])
            transformed_scan = actual.matrix() @ scan_prev
            print(actual)

            plt.scatter(scan_prev[0], scan_prev[1])
            plt.scatter(scan[0], scan[1])
            # plt.scatter(transformed_scan[0], transformed_scan[1])
            plt.legend(["Source scan", "Target scan", "Transformed result"])
            plt.show()

            fig = plt.figure()
            ax = fig.add_subplot(projection='3d')
            ax.scatter(samples[:,0], samples[:,1], samples[:,2], c=np.linspace(0,1,samples.shape[0]), cmap='gist_heat')
            ax.plot(samples[:,0], samples[:,1], samples[:,2], linestyle='dashed')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel(r'$\theta$')
            # ax.set_xlim(0, 0.050)
            # ax.set_ylim(-0.01, 0)
            # ax.set_zlim(-0.014, 0.002)
            plt.show()
            # print(samples)

            break

    bag.close()

if __name__ == "__main__":
    tune_bayesian_icp('data/bags/A1_bag_square_traj_2020-09-04-18-03-08.bag',
                      '/slamware_ros_sdk_server_node/scan',
                      400)