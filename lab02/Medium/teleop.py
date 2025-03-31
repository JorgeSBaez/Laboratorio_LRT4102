#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

def leer_tecla():
    """Captura una tecla presionada sin esperar Enter."""
    descriptor = sys.stdin.fileno()
    config_antigua = termios.tcgetattr(descriptor)
    try:
        tty.setraw(descriptor)
        tecla = sys.stdin.read(1)
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, config_antigua)
    return tecla

def principal():
    rospy.init_node('control_tortuga_teclado', anonymous=True)
    publicador = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    print("\nControl de Tortuga Turtlesim")
    print("----------------------------")
    print("Teclas de movimiento:")
    print("  ↑/w -> Avanzar")
    print("  ↓/s -> Retroceder")
    print("  a   -> Desplazarse a la izquierda")
    print("  d   -> Desplazarse a la derecha")
    print("  q   -> Rotar antihorario")
    print("  e   -> Rotar horario")
    print("  ESPACIO -> Frenar")
    print("Presiona ESC para salir\n")

    velocidad_lineal = 1.5
    velocidad_angular = 1.0

    while not rospy.is_shutdown():
        tecla = leer_tecla()
        movimiento = Twist()
        
        if tecla == 'w' or tecla == '\x1b[A':  # Tecla 'w' o flecha arriba
            movimiento.linear.x = velocidad_lineal
        elif tecla == 's' or tecla == '\x1b[B':  # Tecla 's' o flecha abajo
            movimiento.linear.x = -velocidad_lineal
        elif tecla == 'a':
            movimiento.linear.y = velocidad_lineal
        elif tecla == 'd':
            movimiento.linear.y = -velocidad_lineal
        elif tecla == 'q':
            movimiento.angular.z = velocidad_angular
        elif tecla == 'e':
            movimiento.angular.z = -velocidad_angular
        elif tecla == ' ':
            movimiento.linear.x = 0.0
            movimiento.linear.y = 0.0
            movimiento.angular.z = 0.0
        elif tecla == '\x1b' or tecla == 'x':  # ESC o 'x' para salir
            print("\nFinalizando control...")
            break
        
        publicador.publish(movimiento)

if __name__ == '__main__':
    try:
        principal()
    except rospy.ROSInterruptException:
        pass
    
