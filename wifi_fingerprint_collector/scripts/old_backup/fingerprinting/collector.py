import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#from emergency_stop.msg import EmergencyBreak
import tf
import math

class RobotController:
    def __init__(self):
        rospy.init_node('odom_moving')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.distance_threshold = rospy.get_param('~distance_threshold', 0.5)
        print(self.distance_threshold)
        self.turning = False
        self.rate = rospy.Rate(10)
        self.start_position_x = None
        self.start_position_y = None
        self.last_time = rospy.get_time()

#        self.emergency_msg = EmergencyBreak()
        self.is_stopped = False
        # self.emergency_sub = rospy.Subscriber('/emergency_break', EmergencyBreak, self.emergency_callback)

        self.next_row = -1
        self.y = 0
        self.x = 0
        self.column_length = rospy.get_param('~column_length', 5)
        self.row_length = rospy.get_param('~column_row', 4)

        rospy.loginfo('End configurations')

    def odom_callback(self, msg):
        if self.is_stopped:
            rospy.loginfo("Published emergency braking message: Trigger Signal - {}, Reason - {}"
                      .format(self.emergency_msg.trigger_signal, self.emergency_msg.reason))
            # self.is_stopped = False
            return

        twist = Twist()

        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y

        if self.start_position_x is None:
            self.start_position_x = position_x
            self.start_position_y = position_y

        ### Using transformation matrix to compute the angle
        # orientation_quaternion = (
        #     msg.pose.pose.orientation.x,
        #     msg.pose.pose.orientation.y,
        #     msg.pose.pose.orientation.z,
        #     msg.pose.pose.orientation.w
        # )

        # Convert orientation quaternion to Euler angles
        # euler = tf.transformations.euler_from_quaternion(orientation_quaternion)

        # velocity_x = msg.twist.twist.linear.x
        # velocity_y = msg.twist.twist.linear.y

        distance = math.dist([position_x, position_y], [self.start_position_x, self.start_position_y])

        if self.turning == True:
            twist.linear.x = 0.0
            twist.angular.z = 0.3 * self.next_row
            seconds = rospy.get_time()
            # print("tims diff =", seconds-self.last_time)
            angle = msg.twist.twist.angular.z * (seconds-self.last_time)
            # print("angle =", angle)
            # rospy.loginfo('turing angle={}'.format(angle))

            if abs(angle) >= 1.57: #abs(euler[2])
                rospy.loginfo('turing angle={}'.format(angle))
                self.turning = False  # Reset turning flag
                self.start_position_x = position_x
                self.start_position_y = position_y
                twist.angular.z = 0.0
                rospy.loginfo('will stop turning')
            
            self.pub.publish(twist)
            return
        else:
            # Move the robot forward
            twist.linear.x = 0.2
            self.turning = False
            # self.last_time = rospy.get_time()

        if distance >= self.distance_threshold:
            self.y += self.next_row
            # TODO: collect data
            rospy.loginfo('TODO: collect data')

            if self.y >= self.column_length:
                self.next_row *= -1
                self.x += 1

            self.turning = True
            self.last_time = rospy.get_time()



        self.pub.publish(twist)

'''
    def emergency_callback(self, emergency_msg):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        self.is_stopped = True
        self.emergency_msg = emergency_msg

        rospy.loginfo("Published emergency braking message: Trigger Signal - {}, Reason - {}"
                      .format(emergency_msg.trigger_signal, emergency_msg.reason))

        self.pub.publish(twist)
'''


if __name__ == '__main__':
    controller = RobotController()
    rospy.spin()
