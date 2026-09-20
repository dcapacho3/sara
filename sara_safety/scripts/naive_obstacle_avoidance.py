#!/usr/bin/env python3
# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Nodo de evasión de obstáculos reactiva para el robot SARA
# Implementa un sistema básico de detección y evasión de obstáculos utilizando
# el sensor lidar. El algoritmo monitorea las lecturas de distancia en diferentes
# zonas alrededor del robot y ejecuta una secuencia de movimientos evasivos
# cuando detecta obstáculos en su trayectoria, permitiendo la navegación
# segura en entornos dinámicos.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from enum import Enum

class AvoidanceState(Enum):
   # Estados de la máquina de estados para la evasión
   # Define el comportamiento actual del algoritmo
   IDLE = 0        # Estado inactivo, sin evasión
   BACKING = 1     # Retrocediendo para alejarse del obstáculo
   TURNING = 2     # Girando para cambiar dirección
   FORWARD = 3     # Avanzando en nueva dirección

class ObstacleAvoidanceNode(Node):

   def __init__(self):
       # Inicialización del nodo ROS
       # Configura suscripciones, publicadores y parámetros del sistema
       super().__init__('obstacle_avoidance_node')

       # Configurar QoS Profile para Best Effort
       # Optimiza la comunicación para datos de sensores que pueden perderse
       qos_profile = QoSProfile(
           reliability=QoSReliabilityPolicy.BEST_EFFORT,
           history=QoSHistoryPolicy.KEEP_LAST,
           depth=10
       )

       # Subscripciones
       # Canales de entrada para datos de sensores y comandos actuales
       self.laser_subscription = self.create_subscription(
           LaserScan,
           '/scan',
           self.laser_callback,
           qos_profile)
       
       self.cmd_vel_out_subscription = self.create_subscription(
           Twist,
           '/cmd_vel_out',
           self.cmd_vel_out_callback,
           10)
       
       # Publisher para comandos de evasión
       # Canal de salida para enviar comandos de movimiento evasivo
       self.cmd_vel_publisher = self.create_publisher(
           Twist,
           '/cmd_vel_avoidance',
           10)

       # Control de publicación
       # Variables para gestionar cuándo activar la evasión
       self.publishing_enabled = False
       self.last_publishing_enabled = False
       self.last_cmd_vel_out_msg = Twist()
       self.last_cmd_vel_avoidance_msg = Twist()
       self.should_activate_avoidance = False
       self.avoidance_in_progress = False

       # Ángulos para la detección de obstáculos (en radianes)
       # Define zonas de sensibilidad alrededor del robot SARA
       self.front_angle_range = pi/3  # 60 grados totales en la zona frontal
       self.side_angle_range = pi/6   # 30 grados adicionales a cada lado
       self.rear_angle_min = 3 * pi / 4  # 135 grados
       self.rear_angle_max = 5 * pi / 4  # 225 grados

       # Parámetros de distancia
       # Umbrales para detectar obstáculos relevantes
       self.max_obstacle_distance = 0.4  # Distancia máxima para considerar un obstáculo
       self.min_obstacle_distance = 0.25  # Distancia mínima para filtrar ruido

       # Variables de control
       # Estado actual del sistema de evasión
       self.current_state = AvoidanceState.IDLE
       self.avoidance_side = None  # 'left' o 'right'

       # Parámetros de velocidad
       # Valores de movimiento para cada fase de la evasión
       self.backing_speed = -0.1    # Velocidad de retroceso (m/s)
       self.forward_speed = 0.1     # Velocidad de avance (m/s)
       self.turning_speed = 1.0     # Velocidad de giro (rad/s)

       # Duraciones de las fases (en segundos)
       # Tiempos para cada etapa de la maniobra de evasión
       self.backing_duration = 0.5   # Tiempo de retroceso
       self.turning_duration = 1.5   # Tiempo de giro
       self.forward_duration = 1.0   # Tiempo de avance

       # Timer principal para la máquina de estados
       # Controla la secuencia de evasión
       self.state_timer = self.create_timer(0.1, self.state_machine_callback)
       self.state_start_time = None

   def get_obstacle_zone(self, angle):
    
       #Determina en qué zona se encuentra un ángulo dado:
       #- FRONT: Zona frontal crítica
       #- SIDE_LEFT/SIDE_RIGHT: Zonas laterales de precaución
       #- IGNORE: Zona a ignorar
       #
       # Normalizar el ángulo a [0, 2π]
       # Asegura consistencia en cálculos angulares
       while angle < 0:
           angle += 2 * pi
       while angle >= 2 * pi:
           angle -= 2 * pi

       # Zona frontal central
       # Área de mayor prioridad para detectar obstáculos
       if angle <= self.front_angle_range/2 or angle >= (2*pi - self.front_angle_range/2):
           return "FRONT"
       
       # Zonas laterales frontales
       # Áreas de precaución a los lados del robot
       if angle <= (self.front_angle_range/2 + self.side_angle_range):
           return "SIDE_RIGHT"
       if angle >= (2*pi - (self.front_angle_range/2 + self.side_angle_range)):
           return "SIDE_LEFT"
       
       # Zona trasera (a ignorar)
       # Área donde los obstáculos no afectan el movimiento frontal
       if self.rear_angle_min <= angle <= self.rear_angle_max:
           return "IGNORE"
       
       # Resto de zonas laterales
       # Clasificación de otras áreas alrededor del robot
       if angle < pi:
           return "SIDE_RIGHT"
       return "SIDE_LEFT"

   def cmd_vel_out_callback(self, msg):
       # Monitoreo de comandos de velocidad actuales
       # Determina si el robot está en movimiento para activar la evasión
       if not self.avoidance_in_progress:
           if (msg.linear.x != self.last_cmd_vel_avoidance_msg.linear.x or
               msg.linear.y != self.last_cmd_vel_avoidance_msg.linear.y or
               msg.angular.z != self.last_cmd_vel_avoidance_msg.angular.z):
               
               # Activa la evasión solo si hay movimiento (alguna velocidad no es cero)
               self.should_activate_avoidance = (msg.linear.x != 0.0 or 
                                               msg.linear.y != 0.0 or 
                                               msg.angular.z != 0.0)
           else:
               self.should_activate_avoidance = False

           self.last_cmd_vel_out_msg = msg

   def laser_callback(self, msg):
       # Procesamiento de datos del sensor láser
       # Analiza lecturas para detectar obstáculos en diferentes zonas
       if not self.should_activate_avoidance and not self.avoidance_in_progress:
           self.publishing_enabled = False
           return

       # Variables para el obstáculo más cercano por zona
       # Registra la distancia mínima en cada área relevante
       closest_front = float('inf')
       closest_side_left = float('inf')
       closest_side_right = float('inf')
       
       # Contadores de obstáculos por zona
       # Ayuda a determinar la mejor dirección para evadir
       front_obstacles_count = 0
       left_obstacles_count = 0
       right_obstacles_count = 0
       
       # Analizar cada lectura del láser
       # Procesa el escaneo completo para identificar obstáculos
       for i, range in enumerate(msg.ranges):
           if not (self.min_obstacle_distance < range < self.max_obstacle_distance):
               continue

           current_angle = msg.angle_min + i * msg.angle_increment
           zone = self.get_obstacle_zone(current_angle)
           
           if zone == "IGNORE":
               continue
               
           if zone == "FRONT":
               closest_front = min(closest_front, range)
               front_obstacles_count += 1
           elif zone == "SIDE_LEFT":
               closest_side_left = min(closest_side_left, range)
               left_obstacles_count += 1
           elif zone == "SIDE_RIGHT":
               closest_side_right = min(closest_side_right, range)
               right_obstacles_count += 1

       # Solo iniciar evasión si hay obstáculos en la zona frontal
       # Evita reacciones innecesarias a obstáculos laterales o traseros
       if closest_front < self.max_obstacle_distance:
           if not self.avoidance_in_progress:
               self.avoidance_in_progress = True
               self.publishing_enabled = True
               
               # Determinar lado de evasión basado en obstáculos laterales
               # Elige la dirección con menos obstáculos para girar
               total_left = left_obstacles_count
               total_right = right_obstacles_count
               
               self.avoidance_side = 'izquierda' if total_left > total_right else 'derecha'
               self.get_logger().info(f'Obstáculo frontal detectado. Evadiendo hacia {self.avoidance_side}')
               self.start_avoidance_sequence()
       else:
           if not self.avoidance_in_progress:
               self.publishing_enabled = False

       # Registrar cambios en el estado de publicación
       if self.publishing_enabled != self.last_publishing_enabled:
           self.get_logger().info(f'Publishing {"enabled" if self.publishing_enabled else "disabled"}')
       
       self.last_publishing_enabled = self.publishing_enabled

   def start_avoidance_sequence(self):
       # Inicio de la secuencia de evasión
       # Configura el primer estado y registra el tiempo de inicio
       self.get_logger().info('Iniciando secuencia de evasión')
       self.current_state = AvoidanceState.BACKING
       self.state_start_time = self.get_clock().now()

   def state_machine_callback(self):
       # Máquina de estados para la secuencia de evasión
       # Controla la transición entre los diferentes movimientos
       if not self.avoidance_in_progress:
           return

       # Control de tiempo para cada fase
       current_time = self.get_clock().now()
       if self.state_start_time is None:
           self.state_start_time = current_time

       elapsed_time = (current_time - self.state_start_time).nanoseconds / 1e9

       # Estado de RETROCESO
       # Primera fase: alejarse del obstáculo
       if self.current_state == AvoidanceState.BACKING:
           self.publish_cmd_vel(self.backing_speed, 0.0)
           if elapsed_time >= self.backing_duration:
               self.current_state = AvoidanceState.TURNING
               self.state_start_time = current_time
               self.get_logger().info('Cambiando a estado TURNING')

       # Estado de GIRO
       # Segunda fase: cambiar de dirección
       elif self.current_state == AvoidanceState.TURNING:
           turning_direction = 1.0 if self.avoidance_side == 'izquierda' else -1.0
           self.publish_cmd_vel(0.0, self.turning_speed * turning_direction)
           if elapsed_time >= self.turning_duration:
               self.current_state = AvoidanceState.FORWARD
               self.state_start_time = current_time
               self.get_logger().info('Cambiando a estado FORWARD')

       # Estado de AVANCE
       # Tercera fase: moverse en la nueva dirección
       elif self.current_state == AvoidanceState.FORWARD:
           self.publish_cmd_vel(self.forward_speed, 0.0)
           if elapsed_time >= self.forward_duration:
               self.finish_avoidance()

   def finish_avoidance(self):
       # Finalización de la secuencia de evasión
       # Restablece el estado normal después de completar la maniobra
       self.get_logger().info('Finalizando secuencia de evasión')
       self.current_state = AvoidanceState.IDLE
       self.avoidance_in_progress = False
       self.publishing_enabled = False
       self.state_start_time = None
       self.publish_idle_cmd()

   def publish_cmd_vel(self, linear_x, angular_z):
       # Publicación de comandos de velocidad
       # Envía las órdenes de movimiento al robot SARA
       msg = Twist()
       msg.linear.x = linear_x
       msg.angular.z = angular_z
       msg.angular.y = 0.01  # Mantener roll activo para twist_mux
       self.cmd_vel_publisher.publish(msg)
       self.last_cmd_vel_avoidance_msg = msg


   def publish_idle_cmd(self):
       # Publicación de comando de inactividad
       # Envía velocidad cero pero mantiene el tópico activo
       msg = Twist()
       msg.angular.y = 0.01  # Mantener roll activo para twist_mux
       self.cmd_vel_publisher.publish(msg)
       self.last_cmd_vel_avoidance_msg = msg

def main(args=None):
   # Función principal del programa
   # Inicializa y ejecuta el nodo de evasión de obstáculos
   rclpy.init(args=args)
   node = ObstacleAvoidanceNode()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()