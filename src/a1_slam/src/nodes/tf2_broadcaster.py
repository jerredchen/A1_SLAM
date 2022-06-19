#!/usr/bin/env python  
import rospy

import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped

def handle_pose(msg: PoseStamped):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "body"
    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = 0.0
    q = msg.pose.orientation
    t.transform.rotation.x = q.x
    t.transform.rotation.y = q.y
    t.transform.rotation.z = q.z
    t.transform.rotation.w = q.w

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster')
    rospy.Subscriber('pose_estimate',
                     PoseStamped,
                     handle_pose)
    rospy.spin()