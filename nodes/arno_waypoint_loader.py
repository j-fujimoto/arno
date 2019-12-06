#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Int16

waypoints = PoseArray()
is_insert = -1
pub = None
num = None

def rewriteMarker():
    global num
    marker_data = Marker()
    marker_data.header.frame_id = "map"
    marker_data.header.stamp = rospy.Time.now()
    marker_data.ns = "basic_shapes"
    marker_data.action = Marker.DELETEALL
    num.publish(marker_data)

    marker_data.action = Marker.ADD
    counter = 0
    marker_data.color.a = 1.0
    marker_data.scale.z = 5
    marker_data.lifetime = rospy.Duration()
    marker_data.type = Marker.TEXT_VIEW_FACING
    for pose in waypoints.poses:
        marker_data.id = counter

        marker_data.pose.position.x = pose.position.x
        marker_data.pose.position.y = pose.position.y
        marker_data.pose.orientation.z = pose.orientation.z
        marker_data.pose.orientation.w = pose.orientation.w
        marker_data.text = str(counter)

        num.publish(marker_data)
        counter +=1

def updateWaypointjson():
    file=open('_waypoint.json', 'w')
    file.write("[\n")
    for pose in waypoints.poses:
        file.write("    [[{0},{1},0.0],[0.0,0.0,{2},{3}]],\n".format(pose.position.x,pose.position.y,pose.orientation.z,pose.orientation.w))
    file.write("]")
    file.close()

def printWaypoints():
    print("[")
    for pose in waypoints.poses:
        print("    [[{0},{1},0.0],[0.0,0.0,{2},{3}]],".format(pose.position.x,pose.position.y,pose.orientation.z,pose.orientation.w))
    print("]")

def goalCallback(data):
#pos = data.goal.target_pose.pose
    global pub
    global waypoints
    waypoints.poses.append(data.goal.target_pose.pose)
    printWaypoints()
    updateWaypointjson()
    rewriteMarker()
    pub.publish(waypoints)

def removeCallback(removeId):
    global pub
    global waypoints
    waypoints.poses.pop(removeId.data)
    printWaypoints()
    updateWaypointjson()
    rewriteMarker()
    pub.publish(waypoints)

def insertWaypoint(data):
    global is_insert
    if is_insert != -1:
        waypoints.poses.insert(is_insert, data.goal.target_pose.pose)
        waypoints.poses.pop()
    is_insert = -1

def insertCallback(insertId):
    global pub
    global waypoints
    global is_insert
    is_insert = insertId.data
    print ("insert" + str(is_insert))
    while is_insert != -1:
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, insertWaypoint)
    printWaypoints()
    updateWaypointjson()
    rewriteMarker()
    pub.publish(waypoints)

def listener():
    global pub
    global num
    rospy.init_node('goal_sub', anonymous=True)

    rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, goalCallback)
    rospy.Subscriber("/remove", Int16, removeCallback)
    rospy.Subscriber("/insert", Int16, insertCallback)
    pub = rospy.Publisher('waypoints', PoseArray, queue_size=10)
    num = rospy.Publisher('waypointnumber', Marker, queue_size=10)
    waypoints.header.frame_id = "map"

    rospy.spin()

if __name__ == '__main__':
    listener()
