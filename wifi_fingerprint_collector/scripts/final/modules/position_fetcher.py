#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from wifi_scanner import WiFiScanner
from position_predictor import PositionPredictor


class PositionFetcher():
    def __init__(self):
        self.scanner = WiFiScanner()
        self.predictor = PositionPredictor()
        # self.publisher = rospy.Publisher('position', String, queue_size=10)
        # use the existing position instead of String
        self.publisher = rospy.Publisher('position', Point, queue_size=10)
        self.rate = rospy.Rate(1)

    # this node should predict the position of the robot based on the wifi signal strength then publish it
    def run(self):
        while not rospy.is_shutdown():
            wifi_data = self.scanner.scan_and_get_data()
            predicted_position = self.predictor.predict_position(wifi_data)
            x,y = predicted_position[0]
            # create a new Point object and publish it
            point = Point()
            point.x = x
            point.y = y

            self.publisher.publish(point)
            rospy.loginfo("Position: x: %f, y: %f", point.x, point.y)

            self.rate.sleep()

    
if __name__ == '__main__':
    rospy.init_node('position_fetcher', anonymous=True)
    position_fetcher = PositionFetcher()
    position_fetcher.run()
    rospy.spin()
