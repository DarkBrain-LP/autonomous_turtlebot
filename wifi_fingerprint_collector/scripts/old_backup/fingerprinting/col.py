import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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

        self.is_stopped = False

        self.next_row = -1
        self.y = 0
        self.x = 0
        self.column_length = rospy.get_param('~column_length', 5)
        self.row_length = rospy.get_param('~row_length', 4)
        self.turning_angle = 0.0

        rospy.loginfo('End configurations')

    def odom_callback(self, msg):
        if self.is_stopped:
            return

        twist = Twist()

        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y

        if self.start_position_x is None:
            self.start_position_x = position_x
            self.start_position_y = position_y

        distance = math.dist([position_x, position_y], [self.start_position_x, self.start_position_y])

        if self.turning:
            twist.linear.x = 0.0
            twist.angular.z = 0.3 * self.next_row
            current_time = rospy.get_time()
            time_diff = current_time - self.last_time
            self.turning_angle += abs(0.3 * time_diff)  # Update turning angle

            if self.turning_angle >= (math.pi / 2):
                self.turning = False
                self.turning_angle = 0.0
                self.start_position_x = position_x
                self.start_position_y = position_y
                twist.angular.z = 0.0
                self.next_row *= -1

            self.last_time = current_time
            self.pub.publish(twist)
            return
        else:
            twist.linear.x = 0.2

        if distance >= self.distance_threshold:
            self.y += 1

            if self.y >= self.column_length:
                self.y = 0
                self.x += 1

            self.turning = True
            self.last_time = rospy.get_time()

        self.pub.publish(twist)

if __name__ == '__main__':
    controller = RobotController()
    rospy.spin()
