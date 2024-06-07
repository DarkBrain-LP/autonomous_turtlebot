#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

def move_robot(linear_speed, angular_speed, duration):
    # Initialiser le nœud
    rospy.init_node('move_robot', anonymous=True)

    # Créer un publisher pour le topic cmd_vel
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Créer un message Twist
    vel_msg = Twist()
    vel_msg.linear.x = linear_speed
    vel_msg.angular.z = angular_speed

    # Définir le taux de publication
    rate = rospy.Rate(1)  # 10 Hz

    # Envoi des commandes pendant la durée spécifiée
    start_time = time.time()
    while time.time() - start_time < duration:
        pub.publish(vel_msg)
        rate.sleep()

    # Arrêter le robot après le mouvement
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        # Exemple de paramètres : vitesse linéaire de 0.5 m/s, vitesse angulaire de 0.1 rad/s, durée de 5 secondes
        move_robot(0.5, 0.1, 5)
    except rospy.ROSInterruptException:
        pass
