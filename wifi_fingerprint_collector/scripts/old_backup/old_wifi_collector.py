#!/usr/bin/env python3

import rospy
import subprocess
import re
import csv
import sqlite3
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, pow

class WifiCollector:
    def __init__(self):
        print("Initializing WifiCollector node...")
        rospy.init_node('wifi_collector', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.x = 0
        self.y = 0
        self.init_x = 0
        self.init_y = 0
        self.step_size = 0.5  # 50 cm
        self.grid_width = 3.5  # 350 cm
        self.grid_height = 2.5  # 250 cm
        self.data = []

        self.create_csv()
        self.create_db()
        print("Initialization complete.")

    def create_csv(self):
        print("Creating CSV file for data logging...")
        with open('wifi_data.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(['x', 'y', 's1', 's2', 's3'])
        print("CSV file created.")

    def create_db(self):
        print("Creating SQLite database for data logging...")
        conn = sqlite3.connect('wifi_data.db')
        cursor = conn.cursor()
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS wifi_data (
                x REAL,
                y REAL,
                s1 TEXT,
                s2 TEXT,
                s3 TEXT
            )
        ''')
        conn.commit()
        conn.close()
        rospy.loginfo("SQLite database created.")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rospy.loginfo(f"Odometry updated: x={self.x}, y={self.y}")

    def scan_wifi(self):
        rospy.loginfo("Scanning for Wi-Fi signals...")
        result = subprocess.run(['sudo', 'iwlist', 'wlan0', 'scan'], capture_output=True, text=True)
        lines = result.stdout.split('\n')
        essid = None
        signal = None
        address = None
        signals = [None, None, None]
        target_essids = {
            "0_LP": 0,
            "Yawo": 1,
            "NUMERICABLE-8378": 2,
            "turtlebot": 2,
            "Amirsharata": 2
        }

        for line in lines:
            line = line.strip()
            if line.startswith("Cell"):
                if essid and signal and address:
                    if essid in target_essids:
                        index = target_essids[essid]
                        signals[index] = signal
                address = re.search(r"Address: (\S+)", line).group(1)
                essid = None
                signal = None
            elif "ESSID" in line:
                essid = re.search(r'ESSID:"(.*?)"', line).group(1)
            elif "Signal level" in line:
                signal_match = re.search(r'Signal level=(-?\d+) dBm', line)
                if signal_match:
                    signal = signal_match.group(1)

        if essid and signal and address:
            if essid in target_essids:
                index = target_essids[essid]
                signals[index] = signal

        signals = [signal for signal in signals if signal is not None]
        rospy.loginfo(f"Signals captured: {signals}")
        return signals

    def move_robot(self, linear_speed=0.1, duration=5.0):
        rospy.loginfo(f"Moving robot: linear_speed={linear_speed}, duration={duration}")
        vel_msg = Twist()
        vel_msg.linear.x = linear_speed

        rate = rospy.Rate(10)
        distance_moved = 0.0

        while distance_moved < self.step_size:
            self.cmd_vel_pub.publish(vel_msg)
            rate.sleep()
            distance_moved = sqrt(pow((self.x - self.init_x), 2) + pow((self.y - self.init_y), 2))
            rospy.loginfo(f"Distance moved: {distance_moved}")

        vel_msg.linear.x = 0
        self.cmd_vel_pub.publish(vel_msg)
        rospy.sleep(1)  # Attendre que le robot s'arrête complètement
        rospy.loginfo("Robot stopped.")

    def record_data(self, x, y, s1, s2, s3):
        rospy.loginfo(f"Recording data: x={x}, y={y}, s1={s1}, s2={s2}, s3={s3}")
        with open('wifi_data.csv', 'a') as file:
            writer = csv.writer(file)
            writer.writerow([x, y, s1, s2, s3])

        conn = sqlite3.connect('wifi_data.db')
        cursor = conn.cursor()
        cursor.execute('INSERT INTO wifi_data (x, y, s1, s2, s3) VALUES (?, ?, ?, ?, ?)', (x, y, s1, s2, s3))
        conn.commit()
        conn.close()
        rospy.loginfo("Data recorded.")

    def run(self):
        rospy.loginfo("Starting data collection...")
        rospy.sleep(2)  # Wait for the odometry to be properly initialized

        for y in range(0, int(self.grid_height / self.step_size)):
            for x in range(0, int(self.grid_width / self.step_size)):
                signals = self.scan_wifi()
                s1 = signals[0] if len(signals) > 0 else 'N/A'
                s2 = signals[1] if len(signals) > 1 else 'N/A'
                s3 = signals[2] if len(signals) > 2 else 'N/A'

                self.record_data(self.x, self.y, s1, s2, s3)
                if x < int(self.grid_width / self.step_size) - 1:
                    self.init_x = self.x
                    self.init_y = self.y
                    self.move_robot(linear_speed=0.1, duration=5.0)

            if y < int(self.grid_height / self.step_size) - 1:
                self.init_x = self.x
                self.init_y = self.y
                self.move_robot(linear_speed=0.0, angular_speed=1.57, duration=3.14)  # Tourner à gauche
                self.move_robot(linear_speed=0.1, duration=5.0)  # Avancer de 50 cm
                self.move_robot(linear_speed=0.0, angular_speed=-1.57, duration=3.14)  # Tourner à droite

        rospy.spin()

if __name__ == '__main__':
    try:
        collector = WifiCollector()
        collector.run()
    except rospy.ROSInterruptException:
        pass
