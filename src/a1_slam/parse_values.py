#!/usr/bin/env python

import gtsam
import rospy
import rosbag
from a1_slam.srv import GetResults

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.wait_for_service('get_results_service')
    send_results = rospy.ServiceProxy('get_results_service', GetResults)

    # Sleep to finish obtaining the trajectory before clearing.
    rospy.sleep(1)

    # Obtain the actual poses.
    response = send_results()
    values = gtsam.Values()
    values.deserialize(response.results)

    pose_values = gtsam.utilities.allPose3s(values)
    pose_keys = pose_values.keys()
    bag = rosbag.Bag('/home/jerredchen/A1_trajs_06-04-2022/A1_bag_2022-03-05-07-01-07.bag')
    msgs = list(bag.read_messages('/slamware_ros_sdk_server_node/scan'))
    with open('traj5_a1_slam.txt', 'w') as f:
        for i, pose_key in enumerate(pose_keys):
            pose = pose_values.atPose3(pose_key)
            tx, ty, tz = pose.translation()
            qw, qx, qy, qz = pose.rotation().quaternion()
            ts = msgs[i][1].header.stamp.to_sec()
            if i == 0:
                t_start = ts
            f.write(f"{ts - t_start} {tx} {ty} {tz} {qx} {qy} {qz} {qw}")
            f.write('\n')

if __name__ == '__main__':
    listener()