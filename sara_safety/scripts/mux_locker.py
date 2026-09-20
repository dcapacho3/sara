#!/usr/bin/env python3
# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Controlador del multiplexor de comandos de velocidad para el robot SARA
# Implementa un nodo ROS2 que monitorea datos de sensores externos (principalmente la balanza)
# y genera señales de bloqueo para el multiplexor de comandos de velocidad.
# Esto permite detener el movimiento del robot cuando hay un peso excesivo en la balanza,
# priorizando la seguridad y evitando movimientos indeseados con carga.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
import ast

class TwistMuxController(Node):
   def __init__(self):
       # Inicialización del nodo ROS
       # Configura suscripciones, publicadores y temporizadores para el control del multiplexor
       super().__init__('twist_mux_controller')
       
       # Suscripciones y publicaciones
       # Configura canales de comunicación para sensores y comandos de control
       self.subscription = self.create_subscription(
           String,
           'external_sensor_data',
           self.listener_callback,
           10)
       self.lock_all_publisher = self.create_publisher(Bool, 'lock_all', 10)
       self.lock_navigation_publisher = self.create_publisher(Bool, 'lock_navigation', 10)
       self.cmd_vel_block_all_publisher = self.create_publisher(Twist, 'cmd_vel_block_all', 10)
       self.cmd_vel_block_navigation_publisher = self.create_publisher(Twist, 'cmd_vel_block_navigation', 10)
       
       # Estados de los flags de bloqueo
       # Mantiene el estado actual de las condiciones de bloqueo
       self.lock_all_active = False
       self.lock_navigation_active = False
       
       # Timer para publicar continuamente los mensajes de Twist
       # Asegura que las señales de bloqueo se emitan periódicamente
       self.timer = self.create_timer(0.1, self.publish_velocity)

   def listener_callback(self, msg):
       # Manejo de datos de sensores externos
       # Analiza los datos recibidos de la balanza y otros sensores
       
       # Parsear los datos recibidos
       # Convierte el string recibido en una lista de valores numéricos
       data = ast.literal_eval(msg.data)
       if len(data) != 2:
           self.get_logger().warn('Received data does not contain exactly 2 numbers')
           return
       
       first_number, second_number = data
       
       # Condiciones de bloqueo
       # Define cuándo activar los diferentes modos de bloqueo
       # El primer valor (balanza) determina si se bloquean todos los comandos
       self.lock_all_active = first_number > 1000
       
       # La siguiente línea está en el código pero no es funcional en la implementación actual
       # Se mantiene comentado el segundo valor que originalmente controlaba el bloqueo de navegación
       self.lock_navigation_active = second_number < 700 or second_number > 2000
       
       # Desactivación explícita del bloqueo de navegación (no se utiliza actualmente)
       self.lock_navigation_active = False
       
       # Publicar el estado de los flags
       # Emite señales de bloqueo basadas en el estado actual
       self.publish_lock_all(self.lock_all_active)
       self.publish_lock_navigation(self.lock_navigation_active)

   def publish_lock_all(self, lock):
       # Publicación del flag de bloqueo total
       # Envía mensajes booleanos para activar/desactivar todos los comandos
       msg = Bool()
       msg.data = lock
       self.lock_all_publisher.publish(msg)
       self.get_logger().info(f'Published lock_all: {lock}')

   def publish_lock_navigation(self, lock):
       # Publicación del flag de bloqueo de navegación
       # Envía mensajes booleanos para activar/desactivar comandos de navegación
       # (Actualmente no se utiliza en la implementación final)
       msg = Bool()
       msg.data = lock
       self.lock_navigation_publisher.publish(msg)
       # self.get_logger().info(f'Published lock_navigation: {lock}')

   def publish_velocity(self):
       # Publicación periódica de comandos de velocidad
       # Envía comandos de velocidad cero cuando hay condiciones de bloqueo
       
       # Definir el twist con base en los flags de bloqueo
       # Crea mensajes de velocidad apropiados según el estado de bloqueo
       twist = Twist()
       if self.lock_all_active:
           # Bloqueo total: detener todo movimiento
           # El valor en angular.y es solo para mantener activo el tópico
           twist.linear.x = 0.0
           twist.linear.y = 0.0
           twist.angular.y = 0.005  # Publicar mientras esté en bloqueo total
           self.cmd_vel_block_all_publisher.publish(twist)
       elif self.lock_navigation_active:
           # Bloqueo de navegación: permitir tir teleoperación pero no navegación autónoma
           # (Esta condición no se utiliza actualmente)
           twist.linear.x = 0.0
           twist.linear.y = 0.0
           twist.angular.y = 0.004  # Publicar mientras esté en bloqueo de navegación
           self.cmd_vel_block_navigation_publisher.publish(twist)

def main(args=None):
   # Función principal del programa
   # Inicializa y ejecuta el nodo controlador del multiplexor
   rclpy.init(args=args)
   twist_mux_controller = TwistMuxController()
   rclpy.spin(twist_mux_controller)
   twist_mux_controller.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()