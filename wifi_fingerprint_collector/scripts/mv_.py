import rospy
from geometry_msgs.msg import Twist

# Définir la fonction move_straight()
def move_straight(cmd_vel_pub):
    move_cmd = Twist()
    move_cmd.linear.x = 0.1  # 0.1 m/s
    move_cmd.linear.y = 0.0
    move_cmd.linear.z = 0.0
    move_cmd.angular.x = 0.0
    move_cmd.angular.y = 0.0
    move_cmd.angular.z = 0.0

    # Définir la distance à parcourir
    distance = 0.5  # 50 cm

    # Calculer le temps nécessaire pour parcourir la distance
    time_needed = distance / move_cmd.linear.x

    # Définir la fréquence de publication
    rate = rospy.Rate(10)  # 10 Hz

    # Boucle pour envoyer les commandes de vitesse
    end_time = rospy.Time.now() + rospy.Duration(time_needed)
    while rospy.Time.now() < end_time:
        cmd_vel_pub.publish(move_cmd)
#        rate.sleep()

    # Arrêter le robot après avoir parcouru la distance
    stop_cmd = Twist()
    cmd_vel_pub.publish(stop_cmd)

# Créer le nœud ROS
rospy.init_node('move_robot')

# Créer un éditeur pour le sujet cmd_vel
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Appeler la fonction move_straight() dans la boucle principale du nœud
if __name__ == '__main__':
    try:
        move_straight(cmd_vel_pub)
    except rospy.ROSInterruptException:
        pass
