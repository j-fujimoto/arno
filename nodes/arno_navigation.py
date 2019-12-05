#!/usr/bin/env python

import rospy
import actionlib
import tf
import sys
from nav_msgs.msg import Odometry
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import json
from geometry_msgs.msg import PoseArray, Pose

waypoints = PoseArray()

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose = pose

    return goal_pose

def load_file(fileName):
    with open(fileName, "r") as f:
        waypointFile = json.load(f)
        for line in waypointFile:
            pose = Pose()
            pose.position.x = line[0][0]
            pose.position.y = line[0][1]
            pose.position.z = line[0][2]
            pose.orientation.x = line[1][0]
            pose.orientation.y = line[1][1]
            pose.orientation.z = line[1][2]
            pose.orientation.w = line[1][3]
            waypoints.poses.append(pose)
        waypoints.header.frame_id = "map"
    #print(waypoints)

if __name__ == '__main__':
    rospy.init_node('arno_navigation')
    pub = rospy.Publisher('waypoints', PoseArray, queue_size=10)

    if (len(sys.argv) < 2):
        print("Usage " + sys.argv[0] + " fileName")
        quit()

    load_file(sys.argv[1])
    listener = tf.TransformListener()

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))

    while True:
        for pose in waypoints.poses:
            pub.publish(waypoints)
            goal = goal_pose(pose)
            client.send_goal(goal)
            while True:
                now = rospy.Time.now()
                listener.waitForTransform("map", "base_link", now, rospy.Duration(4.0))

                position, quaternion = listener.lookupTransform("map", "base_link", now)

                if(math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2 ) <= 1):
                    print("next!!")
                    break

                else:
                    rospy.sleep(0.5)
