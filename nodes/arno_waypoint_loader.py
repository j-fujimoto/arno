#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Int16

waypoints = PoseArray()
pub = None

def printWaypoints():
    print("[")
    for pose in waypoints.poses:
        print("[[{0},{1},0.0],[0.0,0.0,{2},{3}]],".format(pose.position.x,pose.position.y,pose.orientation.z,pose.orientation.w))
    print("]")

def goalCallback(data):
#pos = data.goal.target_pose.pose
    global pub
    global waypoints
    waypoints.poses.append(data.goal.target_pose.pose)
    printWaypoints()
    pub.publish(waypoints)

def removeCallback(removeId):
    global pub
    global waypoints
    waypoints.poses.pop()
    printWaypoints()
    pub.publish(waypoints)

def listener():
    global pub
    rospy.init_node('goal_sub', anonymous=True)

    rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, goalCallback)
    rospy.Subscriber("/remove", Int16, removeCallback)
    pub = rospy.Publisher('waypoints', PoseArray, queue_size=10)
    waypoints.header.frame_id = "map"

    rospy.spin()

if __name__ == '__main__':
    listener()
