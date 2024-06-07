#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
import math
from std_srvs.srv import Empty

class Navigator:
    def __init__(self):
        self.destination = Point(5.0, 5.0, 0)  # Destination prédéterminée
        self.current_position = Point()
        
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.position_subscriber = rospy.Subscriber('/position', Point, self.position_callback)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.position_updated = False

    def position_callback(self, data):
        self.current_position = data
        self.position_updated = True
        rospy.loginfo("Current Position: x: %f, y: %f", self.current_position.x, self.current_position.y)

    def navigate_to_destination(self):
        while not rospy.is_shutdown():
            if self.position_updated:
                distance = self.calculate_distance(self.current_position, self.destination)
                angle_to_goal = self.calculate_angle(self.current_position, self.destination)
                
                if distance > 0.1:  # Tolérance de 10 cm
                    self.move_towards_goal(distance, angle_to_goal)
                else:
                    self.stop_robot()
                    rospy.loginfo("Destination atteinte.")
                    break

                self.position_updated = False  # Reset flag after processing
            self.rate.sleep()

    def calculate_distance(self, current_position, destination):
        return math.sqrt((destination.x - current_position.x)**2 + (destination.y - current_position.y)**2)

    def calculate_angle(self, current_position, destination):
        return math.atan2(destination.y - current_position.y, destination.x - current_position.x)

    def move_towards_goal(self, distance, angle_to_goal):
        twist = Twist()
        twist.linear.x = min(0.2, distance)  # Limite la vitesse linéaire à 0.2 m/s
        twist.angular.z = angle_to_goal
        self.velocity_publisher.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.velocity_publisher.publish(twist)

if __name__ == '__main__':
    rospy.init_node('navigator', anonymous=True)
    navigator = Navigator()
    navigator.navigate_to_destination()
    rospy.spin()
