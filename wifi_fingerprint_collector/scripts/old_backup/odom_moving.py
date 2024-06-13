import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class RobotController:
    def __init__(self):
        rospy.init_node('odom_moving')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.distance_threshold = 0.5  # 50 cm
        self.angular_speed = 0.5
        self.linear_speed = 0.1  # Mise à jour de la vitesse linéaire
        self.column_length = rospy.get_param('~column_length', 1.5)
        self.row_length = rospy.get_param('~row_length', 1.5)

        self.turning = False
        self.rate = rospy.Rate(10)
        self.start_position_x = None
        self.start_position_y = None
        self.last_time = rospy.get_time()

        self.column_sum = 0.0
        self.row_sum = 0.0
        self.turning_round = 0
        self.last_orientation = 0

    def odom_callback(self, msg):
        twist = Twist()
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y

        if self.start_position_x is None:
            self.start_position_x = position_x
            self.start_position_y = position_y

        distance = math.dist([position_x, position_y], [self.start_position_x, self.start_position_y])

        if self.turning:
            self.execute_turn(twist, msg)
        else:
            twist.linear.x = self.linear_speed
            if distance >= self.distance_threshold:
                self.update_position(position_x, position_y)
                if self.column_sum >= self.column_length:
                    self.prepare_turn()
                elif self.turning_round == 1:
                    self.prepare_new_row()
        self.pub.publish(twist)

    def execute_turn(self, twist, msg):
        twist.linear.x = 0.0
        twist.angular.z = self.angular_speed * self.last_orientation
        seconds = rospy.get_time()
        angle = msg.twist.twist.angular.z * (seconds - self.last_time)
        if angle >= math.pi:
            self.turning = False
            twist.angular.z = 0.0
            self.last_time = rospy.get_time()  # Reset time for next turn
        self.pub.publish(twist)

    def update_position(self, position_x, position_y):
        self.start_position_x = position_x
        self.start_position_y = position_y
        self.column_sum += self.distance_threshold
        rospy.loginfo(f'x={position_x}, y={position_y} \t column_sum={self.column_sum}')

    def prepare_turn(self):
        self.turning = True
        self.turning_round += 1
        self.column_sum = 0.0
        self.last_time = rospy.get_time()
        rospy.loginfo('Preparing to turn')
        self.last_orientation *= -1  # Change the direction of the next turn

    def prepare_new_row(self):
        self.turning = True
        self.row_sum += self.distance_threshold
        self.turning_round = 0
        self.last_time = rospy.get_time()
        rospy.loginfo('Preparing to turn for new row')
        self.last_orientation *= -1  # Change the direction of the next turn

    def perform_action(self):
        # TODO: Define the action to be performed at each 5cm step
        rospy.loginfo('Performing action at 5cm step')

if __name__ == '__main__':
    controller = RobotController()
    rospy.spin()
