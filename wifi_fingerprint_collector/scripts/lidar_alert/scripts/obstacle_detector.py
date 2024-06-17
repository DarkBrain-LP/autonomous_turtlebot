import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
import math

class LidarObstacleDetector:
    def __init__(self):
        self.laser_scan_sub = rospy.Subscriber("scan", LaserScan, self.laser_scan_msg_callback)
        self.alert_pub = rospy.Publisher("/lidar_alert", Bool, queue_size=10)
        self.angle_pub = rospy.Publisher("/obstacle_angle", Float32, queue_size=10)
        self.scan_data = [0, 0, 0]
        self.obstacle_detected = False  # State to track obstacle detection
        self.latest_scan = None
        self.timer = rospy.Timer(rospy.Duration(3), self.check_for_obstacles)  # Timer to check for obstacles every 3 seconds
    
    def laser_scan_msg_callback(self, msg):
        self.latest_scan = msg

    def check_for_obstacles(self, event):
        if not self.latest_scan:
            return
        
        scan_angle = [0, 30, 330]
        threshold = 0.5  # Threshold distance to detect obstacle
        obstacle_detected = False
        detected_angle = None
        
        for angle in scan_angle:
            distance = self.latest_scan.ranges[angle]
            if distance > 0.0 and distance < threshold:
                obstacle_detected = True
                detected_angle = angle
                print(f"Obstacle detected at angle {angle} with distance {distance}")
                break
        
        if obstacle_detected:
            if not self.obstacle_detected:
                self.obstacle_detected = True
                self.alert_pub.publish(Bool(True))
                self.angle_pub.publish(Float32(detected_angle))
        else:
            if self.obstacle_detected:
                self.obstacle_detected = False
                self.alert_pub.publish(Bool(False))
    
    

if __name__ == '__main__':
    rospy.init_node('lidar_obstacle_detector')
    detector = LidarObstacleDetector()
    rospy.spin()
