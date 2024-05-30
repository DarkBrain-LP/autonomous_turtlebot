#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import subprocess
import re
import csv
import sqlite3

def create_csv():
    print("Creating CSV file for data logging...")
    with open('wifi_data.csv', 'w') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y', 's1', 's2', 's3'])
    print("CSV file created.")

def create_db():
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

def collect_objects(column_length, row_length):
    # Initialize the grid with objects at each node
    grid = [[f"Object({i},{j})" for j in range(row_length)] for i in range(column_length)]
    
    # Initialize the robot's position
    robot_position = (0, 0)
    
    # Initialize a list to keep track of collected objects
    collected_objects = []
    
    for col in range(row_length):
        if col % 2 == 0:  # Moving down the column
            for row in range(column_length):
                robot_position = (row, col)
                collected_objects.append(grid[row][col])
        else:  # Moving up the column
            for row in range(column_length - 1, -1, -1):
                robot_position = (row, col)
                collected_objects.append(grid[row][col])
    
    return collected_objects

def move_robot(pub, linear_x=0, linear_y=0, angular_z=0, duration=1):
    rate = rospy.Rate(10)  # 10hz
    move_cmd = Twist()
    move_cmd.linear.x = linear_x
    move_cmd.linear.y = linear_y
    move_cmd.linear.z = 0
    move_cmd.angular.x = 0
    move_cmd.angular.y = 0
    move_cmd.angular.z = angular_z
    
    for _ in range(int(duration * 10)):
        pub.publish(move_cmd)
        rate.sleep()
    
    stop_cmd = Twist()
    pub.publish(stop_cmd)

def move_straight(cmd_vel_pub):
    move_cmd = Twist()
    move_cmd.linear.x = 0.1  # 0.1 m/s
    move_cmd.linear.y = 0.0
    move_cmd.linear.z = 0.0
    move_cmd.angular.x = 0.0
    move_cmd.angular.y = 0.0
    move_cmd.angular.z = 0.0

    # Définir la fréquence de publication
    rate = rospy.Rate(10)  # 10 Hz

    # Durée pendant laquelle envoyer les commandes
    duration = 5  # secondes

    # Boucle pour envoyer les commandes de vitesse
    end_time = rospy.Time.now() + rospy.Duration(duration)
    while rospy.Time.now() < end_time:
        cmd_vel_pub.publish(move_cmd)
        rate.sleep()

    # Arrêter le robot après avoir parcouru la distance
    stop_cmd = Twist()
    cmd_vel_pub.publish(stop_cmd)

def rotate_robot(pub, angle, angular_speed=0.5):
    duration = abs(angle) / angular_speed
    angular_z = angular_speed if angle > 0 else -angular_speed
    move_robot(pub, angular_z=angular_z, duration=duration)

def collect_column(pub, steps=2, row=0, next_row=-1):
    x = row
    y = 0
    
    for i in range(steps):
        print(i)
        y = i if next_row == -1 else steps-i
        # collect data
        # rospy.loginfo(f'collecting data at pos ({x}, {y})')
        signals = scan_wifi()
        s1 = signals[0] if len(signals) > 0 else 'N/A'
        s2 = signals[1] if len(signals) > 1 else 'N/A'
        s3 = signals[2] if len(signals) > 2 else 'N/A'
        record_data(x, y, s1, s2, s3)
        # move
        move_straight(pub)

    if next_row == -1:
        y += 1
    else:
        y -= 1
        
    signals = scan_wifi()
    s1 = signals[0] if len(signals) > 0 else 'N/A'
    s2 = signals[1] if len(signals) > 1 else 'N/A'
    s3 = signals[2] if len(signals) > 2 else 'N/A'
    record_data(x, y, s1, s2, s3)

    turn_right = True if next_row == -1 else False
    turn_robot(pub, right=turn_right)
    move_straight(pub)
    turn_robot(pub, right=turn_right)


def collect_space_data(pub, column_length=2, row_length=2):
    row_dir = -1
    for row in range(row_length+1):
        collect_column(pub, steps=column_length, row=row, next_row=row_dir)
        row_dir *= -1


def turn_robot(cmd_vel_pub, right=True):
    # Créer un message Twist pour tourner
    rotate_cmd = Twist()
    rotate_cmd.linear.x = 0.0
    rotate_cmd.linear.y = 0.0
    rotate_cmd.linear.z = 0.0
    rotate_cmd.angular.x = 0.0
    rotate_cmd.angular.y = 0.0
    rotate_cmd.angular.z = -0.5 if right else 0.5  # Négatif pour tourner à droite (horaire)

    # Définir la fréquence de publication
    rate = rospy.Rate(10)  # 10 Hz

    # Durée pendant laquelle envoyer les commandes pour tourner
    # Par exemple, pour tourner 90 degrés avec une vitesse angulaire de 0.5 rad/s
    angle_to_turn = 90  # degrés
    angular_speed = 0.5  # rad/s
    duration = (angle_to_turn * 3.14159 / 180) / angular_speed  # Convertir degrés en radians et calculer la durée

    # Boucle pour envoyer les commandes de rotation
    end_time = rospy.Time.now() + rospy.Duration(duration)
    while rospy.Time.now() < end_time:
        cmd_vel_pub.publish(rotate_cmd)
        rate.sleep()

    # Arrêter le robot après avoir tourné
    stop_cmd = Twist()  # Un message Twist avec toutes les valeurs à zéro
    cmd_vel_pub.publish(stop_cmd)

def scan_wifi():
    # rospy.loginfo("Scanning for Wi-Fi signals...")
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
    # rospy.loginfo(f"Signals captured: {signals}")
    return signals


def record_data(x, y, s1, s2, s3):
    rospy.loginfo(f"Recording data: x={x}, y={y}, s1={s1}, s2={s2}, s3={s3}")
    with open('wifi_data.csv', 'a') as file:
        writer = csv.writer(file)
        writer.writerow([x, y, s1, s2, s3])

    conn = sqlite3.connect('wifi_data.db')
    cursor = conn.cursor()
    cursor.execute('INSERT INTO wifi_data (x, y, s1, s2, s3) VALUES (?, ?, ?, ?, ?)', (x, y, s1, s2, s3))
    conn.commit()
    conn.close()
    # rospy.loginfo("Data recorded.")


def main():
    rospy.init_node('robot_collector', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    create_csv()
    create_db()

    # move_straight(pub)
    # turn_robot(pub)
    # collect_column(pub, steps=2, row=0, next_row=1)
    collect_space_data(pub)

    # move_robot(pub, linear_x=0.5, duration=1) 
    # column_length = rospy.get_param('~column_length', 5)  # Example column length (number of rows)
    # row_length = rospy.get_param('~row_length', 4)        # Example row length (number of columns)
    
    # objects = collect_objects(column_length, row_length)
    # rospy.loginfo("Collected objects: %s", objects)

    # current_direction = 'down'
    
    # for col in range(row_length):
    #     if current_direction == 'down':
    #         for _ in range(column_length):
    #             rospy.loginfo("Moving down in column %d", col)
    #             move_robot(pub, linear_x=0.5, duration=1)  # Move down
    #         current_direction = 'up'
    #     else:
    #         for _ in range(column_length):
    #             rospy.loginfo("Moving up in column %d", col)
    #             move_robot(pub, linear_x=-0.5, duration=1)  # Move up
    #         current_direction = 'down'

    #     # Move to the next column
    #     if col < row_length - 1:
    #         rospy.loginfo("Rotating to move to the next column %d", col + 1)
    #         rotate_robot(pub, math.pi / 2)  # Rotate 90 degrees to the right
    #         move_robot(pub, linear_x=0.5, duration=1)  # Move to the next column
    #         rotate_robot(pub, -math.pi / 2)  # Rotate 90 degrees back to face the rows

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
