#!/usr/bin/env python3
import rospy
import math
import time  
import numpy as np
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, TeleportAbsolute, Kill
from std_msgs.msg import String

class RobotGuiadoInteligente:
    def __init__(self):
        rospy.init_node('robot_guiado_inteligente')
        
        # Configuración inicial
        self.configurar_entorno()
        
        # Variables de estado
        self.pose = None
        self.objetivo = None
        self.estado = "ESPERA"
        self.metricas = {
            'tiempo_inicio': 0,
            'distancia_recorrida': 0,
            'obstaculos_evitados': 0
        }
        
        # Umbrales de seguridad
        self.umbral_obstaculo = 1.5
        self.velocidad_segura = 0.3
        
        # Configuración de comunicación
        self.pub_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pub_feedback = rospy.Publisher('/feedback_robot', String, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, self.actualizar_pose)
        rospy.Subscriber('/comando_voz', String, self.procesar_comando)
        
        rospy.loginfo("Sistema de guiado inteligente inicializado")

    def configurar_entorno(self):
        """Prepara el entorno de simulación con sillas y robot"""
        try:
            rospy.wait_for_service('/kill')
            rospy.wait_for_service('/spawn')
        
            kill = rospy.ServiceProxy('/kill', Kill)
            kill('turtle1')
        
            spawn = rospy.ServiceProxy('/spawn', Spawn)
            spawn(1.0, 1.0, 0, 'turtle1')
        
            # Crear sillas (obstáculos y destinos) 
            self.sillas = {
                '1': (2.0, 2.0),
                '2': (5.0, 2.0),
                '3': (8.0, 2.0),
                '4': (2.0, 5.0),
                '5': (5.0, 5.0),
                '6': (8.0, 5.0),
                '7': (2.0, 8.0),  
                '8': (5.0, 8.0),  
                '9': (8.0, 8.0)   
            }
        
            for id, pos in self.sillas.items():
                spawn(pos[0], pos[1], 0, f'silla_{id}')
            
        except Exception as e:
            rospy.logerr(f"Error configurando entorno: {e}")

    def actualizar_pose(self, data):
        """Actualiza la posición actual y calcula métricas"""
        if self.pose:
            self.metricas['distancia_recorrida'] += math.sqrt(
                (data.x - self.pose.x)**2 + (data.y - self.pose.y)**2
            )
        self.pose = data

    def procesar_comando(self, msg):
        """Interpreta comandos de voz para navegación"""
        comando = msg.data.lower()
        
        if "silla" in comando:
            try:
                num_silla = comando.split("silla")[1].strip()
                if num_silla in self.sillas:
                    self.objetivo = self.sillas[num_silla]
                    self.estado = "NAVEGANDO"
                    self.metricas = {
                        'tiempo_inicio': time.time(),  
                        'distancia_recorrida': 0,
                        'obstaculos_evitados': 0
                    }
                    self.enviar_feedback(f"Iniciando navegación a silla {num_silla}")
                else:
                    self.enviar_feedback(f"Silla {num_silla} no reconocida")
                    
            except Exception as e:
                rospy.logerr(f"Error procesando comando: {e}")
                self.enviar_feedback("Error procesando comando")
                
        elif "detener" in comando:
            self.detener()
            self.enviar_feedback("Navegación detenida")

    def navegar_con_evasion(self):
        """Lógica de navegación con evitación de obstáculos"""
        if not self.pose or not self.objetivo or self.estado != "NAVEGANDO":
            return
        
        dx = self.objetivo[0] - self.pose.x
        dy = self.objetivo[1] - self.pose.y
        distancia = math.hypot(dx, dy)
        angulo_deseado = math.atan2(dy, dx)
        
        twist = Twist()
        
        if distancia < 0.2:
            self.detener()
            tiempo = time.time() - self.metricas['tiempo_inicio']
            self.enviar_feedback(
                f"Destino alcanzado. Tiempo: {tiempo:.1f}s, "
                f"Distancia: {self.metricas['distancia_recorrida']:.2f}m, "
                f"Obstáculos evitados: {self.metricas['obstaculos_evitados']}"
            )
            return
        
        fuerza_evasion = self.calcular_evasion()
        
        if fuerza_evasion:
            twist.linear.x = self.velocidad_segura
            twist.angular.z = 2.0 * fuerza_evasion
            self.metricas['obstaculos_evitados'] += 0.1
        else:
            diff_angulo = angulo_deseado - self.pose.theta
            diff_angulo = math.atan2(math.sin(diff_angulo), math.cos(diff_angulo))
            
            twist.linear.x = min(0.5 * distancia, 1.0)
            twist.angular.z = 1.5 * diff_angulo
            
            if distancia < 1.0:
                self.enviar_feedback(f"A {distancia:.1f}m del destino")

        self.pub_vel.publish(twist)

    def calcular_evasion(self):
        """Calcula fuerza de evasión basada en obstáculos cercanos"""
        fuerza = 0.0
        
        for id, silla in self.sillas.items():
            if silla == self.objetivo:
                continue
                
            dist = math.hypot(silla[0]-self.pose.x, silla[1]-self.pose.y)
            if dist < self.umbral_obstaculo:
                angulo_obstaculo = math.atan2(
                    self.pose.y - silla[1], 
                    self.pose.x - silla[0]
                )
                peso = (self.umbral_obstaculo - dist) / self.umbral_obstaculo
                fuerza += peso * math.sin(angulo_obstaculo - self.pose.theta)
                
                if dist < 1.0:
                    self.enviar_feedback(f"Evitando silla {id}")

        return fuerza

    def detener(self):
        """Detiene completamente el robot"""
        twist = Twist()
        self.pub_vel.publish(twist)
        self.estado = "ESPERA"
        self.objetivo = None

    def enviar_feedback(self, mensaje):
        """Envía retroalimentación al usuario"""
        self.pub_feedback.publish(String(mensaje))
        rospy.loginfo(mensaje)

    def ejecutar(self):
        """Bucle principal de control"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.estado == "NAVEGANDO":
                self.navegar_con_evasion()
            rate.sleep()

if __name__ == '__main__':
    try:
        robot = RobotGuiadoInteligente()
        robot.ejecutar()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo finalizado")
