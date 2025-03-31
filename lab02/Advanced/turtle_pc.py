#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pi

class MoveTurtleProportionalControl:
    def __init__(self):
        rospy.init_node('control_tortuga_xytheta')
        
        # Suscribirse al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el topic de comandos de movimiento de la tortuga
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Variables para almacenar la posición actual de la tortuga
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

        # Constantes de proporcionalidad 
        self.Kp_linear = 1.0  # Ganancia para el control lineal (x, y)
        self.Kp_angular = 4.0  # Ganancia para el control angular (theta)


    def pose_callback(self, pose):
        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta


    def move_turtle_to_desired_pose(self, desired_x, desired_y, desired_theta):
        while not rospy.is_shutdown():
            # Calcular los errores
            error_x = desired_x - self.current_x
            error_y = desired_y - self.current_y
            error_theta = atan2(error_y, error_x) - self.current_theta

            # Normalizar el error angular para evitar discontinuidades
            if error_theta > pi:
                error_theta -= 2 * pi
            elif error_theta < -pi:
                error_theta += 2 * pi

            # Calcular la distancia lineal hacia el objetivo
            distance_to_goal = sqrt(error_x**2 + error_y**2)

            # Control proporcional para la velocidad lineal y angular
            vel_linear = self.Kp_linear * distance_to_goal
            vel_angular = self.Kp_angular * error_theta

            # Crear un mensaje de Twist para enviar el comando de movimiento
            twist_msg = Twist()
            twist_msg.linear.x = vel_linear
            twist_msg.angular.z = vel_angular

            # Publicar el mensaje
            self.velocity_publisher.publish(twist_msg)

            # Imprimir la posición actual, el error y las velocidades en la terminal
            rospy.loginfo("Posición actual: (%f, %f, %f)", self.current_x, self.current_y, self.current_theta)
            rospy.loginfo("Error: (%f, %f, %f)", error_x, error_y, error_theta)
            rospy.loginfo("Velocidad lineal: %f, Velocidad angular: %f", vel_linear, vel_angular)

            # Verificar si se alcanza la posición deseada
            if distance_to_goal < 0.1 and abs(error_theta) < 0.1:
                rospy.loginfo("Posición deseada alcanzada")
                break

            # Esperar hasta la siguiente iteración
            self.rate.sleep()


    def get_desired_pose_from_user(self):
        print("Ingrese la posición deseada:")
        desired_x = float(input("Coordenada x: "))
        desired_y = float(input("Coordenada y: "))
        desired_theta = float(input("Orientación theta (en radianes): "))
        return desired_x, desired_y, desired_theta


    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Obtener la posición deseada del usuario
            desired_x, desired_y, desired_theta = self.get_desired_pose_from_user()

            # Mover la tortuga a la posición deseada
            self.move_turtle_to_desired_pose(desired_x, desired_y, desired_theta)


if __name__ == '__main__':
    try:
        move_turtle_proportional = MoveTurtleProportionalControl()
        move_turtle_proportional.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
