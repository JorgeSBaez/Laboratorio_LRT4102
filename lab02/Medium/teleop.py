#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Kill, Spawn
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

def kill_turtle():
    """Mata la tortuga actual."""
    rospy.wait_for_service('/kill')
    try:
        kill = rospy.ServiceProxy('/kill', Kill)
        kill('turtle1')  # Mata la tortuga llamada 'turtle1'
    except rospy.ServiceException as e:
        print(f"Error al matar la tortuga: {e}")

def spawn_turtle(x, y, theta):
    """Crea una nueva tortuga en la posición (x, y) con orientación theta."""
    rospy.wait_for_service('/spawn')
    try:
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(x, y, theta, 'turtle1')  # Crea una nueva tortuga llamada 'turtle1'
    except rospy.ServiceException as e:
        print(f"Error al crear la tortuga: {e}")

def main():
    rospy.init_node('turtle_keyboard_control', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    print("Controla la tortuga con las teclas:")
    print("  c -> Dibujar un cuadrado")
    print("  t -> Dibujar un triángulo")
    print("Presiona 'q' para salir.")

    while not rospy.is_shutdown():
        key = get_key()
        
        if key == 'q':  
            print("Saliendo...")
            break  # Sale del loop

        elif key == 'c':
            print("Dibujando un cuadrado...")
            draw_square(pub)
            print("Matando la tortuga...")
            kill_turtle()
            print("Creando una nueva tortuga...")
            spawn_turtle(5.5, 5.5, 0.0)  # Crea una nueva tortuga en el centro
            print("Listo para dibujar un triángulo.")

        elif key == 't':
            print("Dibujando un triángulo...")
            draw_triangle(pub)

if __name__ == '__main__':
    main()