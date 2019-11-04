#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry

def odometryCallback(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
            msg.header.stamp, "base_link", "odom")

if __name__ == '__main__':
    print("start odom_tf_publisher")
    rospy.init_node('odom_tf_publisher')
    rospy.Subscriber('odom', Odometry, odometryCallback)
    rospy.spin()
