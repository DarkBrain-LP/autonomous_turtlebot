#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from scheduler.msg import active

wifi_position = Pose()
slam_position = Pose()
target_position = PoseStamped()
wifi_node = active()

deactivate = active()
deactivate.active = False

activate = active()
activate.active = True

# Goal coordinates
goal = Pose()
if len(sys.argv) < 5:
    goal.position.x = 6.1220316886901855
    goal.position.y = -4.283902645111084
    goal.position.z = 0.0
    goal.orientation.x = 0.0
    goal.orientation.y = 0.0
    goal.orientation.z = 0.999920922405971
    goal.orientation.w = 0.01257572800246908
else:
    goal.position.x = float(sys.argv[1])
    goal.position.y = float(sys.argv[2])
    goal.position.z = 0.0
    goal.orientation.x = 0.0
    goal.orientation.y = 0.0
    goal.orientation.z = float(sys.argv[3])
    goal.orientation.w = float(sys.argv[4])

def get_wifi_position(data):
    global wifi_position
    wifi_position = data

def get_slam_position(data):
    global slam_position
    slam_position = data

def get_target_position(data):
    global target_position
    target_position = data

    navigation_node_pub.publish(deactivate)

def get_wifi_node(data):
    global wifi_node
    wifi_node = data

    if not wifi_node.active:
        starting_point_pub.publish(wifi_position)
        rospy.sleep(0.5)
        navigation_node_pub.publish(activate)

wifi_node_pub = rospy.Publisher('/wifi', active, queue_size=10)
navigation_node_pub = rospy.Publisher('/navigation', active, queue_size=10)
starting_point_pub = rospy.Publisher('/starting_position', Pose, queue_size=10)
goal_pub = rospy.Publisher('/goal', Pose, queue_size=10)

wifi_node_sub = rospy.Subscriber('/wifi', active, get_wifi_node)

wifi_position_sub = rospy.Subscriber('/wifi_position', Pose, get_wifi_position)
slam_position_sub = rospy.Subscriber('/slam_position', Pose, get_slam_position)
camera_target_sub = rospy.Subscriber('/camera_target', PoseStamped, get_target_position)

if __name__ == '__main__':
    try:
        rospy.init_node('scheduler_node')
        rospy.sleep(1)
        # Activate the wifi node
        wifi_node_pub.publish(activate)
        # Deactivate the navigation node
        navigation_node_pub.publish(deactivate)
        # Publish the goal
        goal_pub.publish(goal)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass