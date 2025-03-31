#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

def get_key():
    """Lee una tecla del teclado sin necesidad de presionar Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)  # Lee un solo caracter
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def move_to_new_position(pub):
    """Mueve la tortuga a una nueva posición para evitar superposiciones."""
    msg = Twist()
    msg.linear.x = 2.0  # Avanza en X
    pub.publish(msg)
    rospy.sleep(2)  # Avanza durante 2 segundos
    msg.linear.x = 0.0  # Detiene el movimiento
    pub.publish(msg)

def draw_square(pub):
    """Hace que la tortuga dibuje un cuadrado."""
    msg = Twist()
    for _ in range(4):
        msg.linear.x = 2.0  # Avanza
        pub.publish(msg)
        rospy.sleep(2)
        msg.linear.x = 0.0  # Detiene el movimiento lineal
        msg.angular.z = 1.57  # Gira 90 grados (en radianes)
        pub.publish(msg)
        rospy.sleep(1)
        msg.angular.z = 0.0  # Detiene la rotación
    pub.publish(msg)  # Asegura que la tortuga se detenga

def draw_triangle(pub):
    """Hace que la tortuga dibuje un triángulo."""
    msg = Twist()
    for _ in range(3):
        msg.linear.x = 2.0  # Avanza
        pub.publish(msg)
        rospy.sleep(2)
        msg.linear.x = 0.0  # Detiene el movimiento lineal
        msg.angular.z = 2.09  # Gira 120 grados (en radianes)
        pub.publish(msg)
        rospy.sleep(1)
        msg.angular.z = 0.0  # Detiene la rotación
    pub.publish(msg)  # Asegura que la tortuga se detenga

def main():
    rospy.init_node('turtle_keyboard_control', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    print("Controla la tortuga con las teclas:")
    print("Presiona 'q' para salir.")
    print("Presiona 'c' para hacer un cuadrado.")
    print("Presiona 't' para hacer un triángulo.")

    while not rospy.is_shutdown():
        key = get_key()
        msg = Twist()
        
        if key == 'q':  
            print("Saliendo...")
            break  # Sale del loop

        elif key == 'c':
            print("Moviendo a nueva posición...")
            move_to_new_position(pub)  # Mueve la tortuga a una nueva posición
            print("Haciendo Cuadrado...")
            draw_square(pub)

        elif key == 't':
            print("Moviendo a nueva posición...")
            move_to_new_position(pub)  # Mueve la tortuga a una nueva posición
            print("Haciendo Triángulo...")
            draw_triangle(pub)
        
        pub.publish(msg)

if __name__ == '__main__':
    main()
