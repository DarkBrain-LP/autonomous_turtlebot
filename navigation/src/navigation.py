#!/usr/bin/env python
 
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from scheduler.msg import active
 
navigation_node = active()
goal = Pose()
goal_is_ready = False
starting_position = Pose()

def get_navigation_node(data):
    global navigation_node, goal_is_ready
    navigation_node = data
    goal_is_ready = True

def get_goal(data):
    global goal
    goal = data

def get_starting_position(data):
    global starting_position
    starting_position = data

def init_pose(initialpose, cmd_vel, start_position):
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rospy.Time.now()
    pose.pose.pose.position.x = start_position[0]
    pose.pose.pose.position.y = start_position[1]
    pose.pose.pose.orientation.z = start_position[2]
    pose.pose.pose.orientation.w = start_position[3]
    pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
    initialpose.publish(pose)
    rospy.sleep(0.3)
    vel = Twist()
    vel.angular.z = 1
    cmd_vel.publish(vel)
    rospy.sleep(2)
    vel.angular.z = 0
    cmd_vel.publish(vel)

 
def init_goal(waypoints):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = waypoints[0]
    goal.target_pose.pose.position.y = waypoints[1]
    goal.target_pose.pose.orientation.z = waypoints[2]
    goal.target_pose.pose.orientation.w = waypoints[3]
    return goal
 
if __name__ == '__main__':
    try:
        # Initialize the node
        rospy.init_node('navigation_node')
        # Initialize the publishers and the client
        initialpose = rospy.Publisher ('/initialpose', PoseWithCovarianceStamped, queue_size = 10)
        cmd_vel = rospy.Publisher ('/cmd_vel', Twist, queue_size=10)

        navigation_node_sub = rospy.Subsciber('/navigation', active, get_navigation_node)
        goal_sub = rospy.Subsciber('/goal', Pose, get_goal)
        starting_position_sub = rospy.Subsciber('/starting_position', Pose, get_starting_position)
        rospy.sleep(1)

        client = SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        # Getting the goal from the scheduler
        while not goal_is_ready:
            pass

        waypoint = [goal.position.x, goal.position.y, goal.orientation.z, goal.orientation.w]

        while not navigation_node.active:
            pass

        # Initialize the starting position
        startpoint = [starting_position.position.x, starting_position.position.y, starting_position.orientation.z, starting_position.orientation.w]
        init_pose(initialpose, cmd_vel, startpoint)

        goal = init_goal(waypoint)
        rospy.loginfo("Sending goal: %s", goal)
        client.send_goal(goal)
        while navigation_node.active:
            pass
        
    except rospy.ROSInterruptException:
        pass