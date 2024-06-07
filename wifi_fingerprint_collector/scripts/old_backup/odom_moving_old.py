import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class RobotController:
    def __init__(self):
        rospy.init_node('odom_moving')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.distance_threshold = 0.5 # rospy.get_param('~distance_threshold', 0.5)

        self.turning = False
        self.rate = rospy.Rate(10)
        self.start_position_x = None
        self.start_position_y = None
        self.last_time = rospy.get_time()
        
        self.is_stopped = False

        self.column_length = 1.5
        self.row_length = 1.5
        self.column_sum = 0.0
        self.row_sum = 0.0
        self.end_column = False
        self.end_row = False
         
        self.turning_round = 0

        

    def odom_callback(self, msg):
        # if self.is_stopped:
        #     rospy.loginfo("Published emergency braking message: Trigger Signal - {}, Reason - {}"
        #               .format(self.emergency_msg.trigger_signal, self.emergency_msg.reason))
        #     # self.is_stopped = False
        #     return
        
        twist = Twist()

        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y

        if self.start_position_x is None:
            self.start_position_x = position_x
            self.start_position_y = position_y
        
        distance = math.dist([position_x, position_y], [self.start_position_x, self.start_position_y])

        if self.turning == True:
            rospy.loginfo("turning !")
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            seconds = rospy.get_time()
            angle = msg.twist.twist.angular.z * (seconds-self.last_time)

            if angle >= 1.57: #abs(euler[2]) 
                self.turning = False  # Reset turning flag 
                # self.start_position_x = position_x
                # self.start_position_y = position_y
                twist.angular.z = 0.0
            self.pub.publish(twist)
            return
        else:
            # Move the robot forward
            twist.linear.x = 0.1
            self.turning = False 
            # self.last_time = rospy.get_time()
        
        if distance >= self.distance_threshold:
            self.start_position_x = position_x
            self.start_position_y = position_y
            # collect data here
            self.column_sum += self.distance_threshold
            rospy.loginfo(f'x={position_x}, y={position_y} \t column_sum={self.column_sum}, distance_threshold={self.distance_threshold}')

            if self.column_sum >= self.column_length:
                rospy.loginfo(f'column_sum={self.column_sum}, column_length={self.column_length} -> turning')
                self.turning = True
                self.turning_round += 1
                self.column_sum = 0.0
                self.last_time = rospy.get_time()

                return
            
            rospy.loginfo(f'turning={self.turning_round}')
            if self.turning_round == 1:
                rospy.loginfo(f'should turn next')
                self.turning = True
                self.row_sum += self.distance_threshold
                self.turning_round = 0

                return

        

        
        self.pub.publish(twist)


    # def emergency_callback(self, emergency_msg):
    #     twist = Twist()
    #     twist.linear.x = 0.0
    #     twist.angular.z = 0.0

    #     self.is_stopped = True
    #     self.emergency_msg = emergency_msg

    #     rospy.loginfo("Published emergency braking message: Trigger Signal - {}, Reason - {}"
    #                   .format(emergency_msg.trigger_signal, emergency_msg.reason))
        
    #     self.pub.publish(twist)



if __name__ == '__main__':
    controller = RobotController()
    rospy.spin()
