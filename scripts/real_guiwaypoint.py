#!/usr/bin/env python3
# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Sistema de navegación autónoma para el robot SARA en entorno de supermercado
# Implementa un nodo de navegación que planifica y ejecuta rutas optimizadas para
# la recolección de productos en un supermercado. El sistema consulta una base de datos
# de productos seleccionados, calcula la ruta más corta utilizando algoritmos de optimización,
# y guía al robot a través de los puntos de interés, esperando confirmación en cada parada
# para continuar con la siguiente ubicación.

import sqlite3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from robot_navigator import BasicNavigator, NavigationResult
import math
import numpy as np
import yaml
from PIL import Image
import os
from itertools import permutations
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_from_euler

def get_source_db_path(package_name, db_filename):
   
   #Obtiene la ruta a la base de datos de productos en el directorio src del paquete
   #Facilita el acceso a la información de ubicación de productos
   
   # Obtener el directorio share del paquete
   share_dir = get_package_share_directory(package_name)
   
   # Navegar hasta la raíz del workspace (subir 4 niveles: share/package/install/workspace)
   workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(share_dir))))
   
   # Construir la ruta a la base de datos en src
   db_path = os.path.join(workspace_root, 'src', package_name, 'database', db_filename)
   
   return db_path

class AutonomousNavigator:
   def __init__(self):
       # Inicialización del nodo y configuración de componentes
       # Establece las conexiones necesarias para la navegación y monitoreo
       self.node = rclpy.create_node('navigator_node')
       self.odom_subscriber = OdomSubscriber(self.node)
       self.continue_subscriber = ContinueSubscriber(self.node) 
       self.todonext_subscriber = ToDoNextSubscriber(self.node)
       self.status_publisher = self.node.create_publisher(String, '/navigation_status', 10)
       self.visited_waypoints = set()
       
       # Inicialización del navegador básico
       # Conecta con el sistema Nav2 para control de movimiento
       self.navigator = BasicNavigator()
       self.navigator.waitUntilNav2Active()
       
       # Ubicación predefinida de la caja registradora
       # Punto final fijo para todas las rutas de recolección
       self.fixed_cash_location = {'name': 'cashier', 'x': 0.1, 'y': 2.0}

   def publish_status(self, status, waypoint_name=None, completion_percentage=None):
       
       #Publica el estado actual de la navegación
       #Proporciona retroalimentación sobre el progreso y ubicación del robot
       
       msg = String()
       msg.data = f"{status}|{waypoint_name or ''}|{completion_percentage or ''}|{','.join(self.visited_waypoints)}"
       self.status_publisher.publish(msg)

   def get_product_locations(self):
       
       #Obtiene las ubicaciones de productos seleccionados de la base de datos
       #Recupera las coordenadas de los productos que el robot debe visitar
       
       db_dir = get_source_db_path('turtlemart', 'products.db')
       conn = sqlite3.connect(db_dir)
       cursor = conn.cursor()
       cursor.execute('SELECT name, x, y FROM selected_products')
       locations = [{'name': row[0], 'x': row[1], 'y': row[2]} for row in cursor.fetchall()]
       
       conn.close()
       locations.append(self.fixed_cash_location)
    
       return locations

   def calculate_distance(self, p1, p2):
       
       #Calcula la distancia euclidiana entre dos puntos
       #Utilizado para medir distancias entre ubicaciones y optimizar rutas
      
       return math.sqrt((p1['x'] - p2['x'])**2 + (p1['y'] - p2['y'])**2)

   def nearest_neighbor_with_fixed_end(self, start, locations, end):

       #Implementa el algoritmo del vecino más cercano con punto final fijo
       #Genera una ruta inicial visitando siempre el punto más cercano al actual
 
       current = start
       unvisited = locations.copy()
       path = [start]
       
       while unvisited:
           # Encontrar el punto más cercano al punto actual
           nearest = min(unvisited, key=lambda x: self.calculate_distance(current, x))
           path.append(nearest)
           current = nearest
           unvisited.remove(nearest)
       
       # Agregar el punto final (caja)
       path.append(end)
       return path

   def two_opt_swap(self, path, i, j):
       
       #Realiza un intercambio 2-opt en la ruta
       #Invierte un segmento de la ruta para intentar mejorarla
       
       return path[:i] + path[i:j+1][::-1] + path[j+1:]

   def two_opt_improvement(self, path):
       
       #Aplica la heurística 2-opt para mejorar la ruta
       #Busca iterativamente intercambios que reduzcan la distancia total
       
       improvement = True
       best_distance = self.calculate_total_distance(path)
       while improvement:
           improvement = False
           for i in range(1, len(path) - 2):
               for j in range(i + 1, len(path) - 1):
                   new_path = self.two_opt_swap(path, i, j)
                   new_distance = self.calculate_total_distance(new_path)
                   if new_distance < best_distance:
                       path = new_path
                       best_distance = new_distance
                       improvement = True
       return path
       
   def calculate_total_distance(self, path):
    
       #Calcula la distancia total de una ruta
       #Suma las distancias entre puntos consecutivos del camino
     
       return sum(self.calculate_distance(path[i], path[i+1]) for i in range(len(path) - 1))
       
   def optimize_path(self, start, locations, end):
       
       #Optimiza la ruta utilizando una combinación de algoritmos
       #Genera una ruta eficiente para visitar todos los productos seleccionados
      
       initial_path = self.nearest_neighbor_with_fixed_end(start, locations, end)
       optimized_path = self.two_opt_improvement(initial_path)
       return optimized_path

   def calculate_completion_percentage(self, previous_pose, current_pose, goal_pose):
       
       #Calcula el porcentaje de completado de un tramo de navegación
       #Proporciona retroalimentación sobre el progreso hacia el objetivo actual
      
       total_distance = self.calculate_distance(previous_pose, goal_pose)
       remaining_distance = self.calculate_distance(current_pose, goal_pose)
       completion_percentage = ((total_distance - remaining_distance) / total_distance) * 100
       return max(0, min(completion_percentage, 100))

   def navigate(self):
       
       #Función principal de navegación
       #Coordina todo el proceso de planificación y ejecución de la ruta
    
       # Esperar a recibir la posición inicial del robot
       print("Waiting for initial position from odometry...")
       while self.odom_subscriber.current_pose is None:
           rclpy.spin_once(self.node)
       start_pose = self.odom_subscriber.current_pose
       print(f"Initial position: {start_pose}")

       # Obtener ubicaciones de productos y verificar que haya productos seleccionados
       locations = self.get_product_locations()
       if not locations:
           print("No selected products found.")
           return

       # Separar la ubicación de la caja registradora de los productos
       cashier = next(loc for loc in locations if loc['name'] == 'cashier')
       other_locations = [loc for loc in locations if loc['name'] != 'cashier']

       # Calcular la ruta óptima para visitar todos los productos
       optimized_path = self.optimize_path(start_pose, other_locations, cashier)

       # Solicitar confirmación del usuario antes de iniciar la navegación
       self.publish_status("READY")
       while not self.continue_subscriber.should_continue:
           rclpy.spin_once(self.node)
       
       # Preparar las poses objetivo para cada punto de la ruta
       goal_poses = []
       pose_names = {}

       for i, loc in enumerate(optimized_path[1:-1]):  # Omitir start_pose y end_pose
           goal_pose = PoseStamped()
           goal_pose.header.frame_id = 'map'
           goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
           goal_pose.pose.position.x = loc['x']
           goal_pose.pose.position.y = loc['y']
           current_pose = self.odom_subscriber.current_pose
           if current_pose:
               # Calcular orientación para mirar hacia el objetivo
               yaw = self.calculate_goal_orientation(current_pose['x'], current_pose['y'], loc['x'], loc['y'])
               q = quaternion_from_euler(0, 0, yaw)
               goal_pose.pose.orientation.x = q[0]
               goal_pose.pose.orientation.y = q[1]
               goal_pose.pose.orientation.z = q[2]
               goal_pose.pose.orientation.w = q[3]
           else:
               goal_pose.pose.orientation.w = 1.0  # Cuaternión identidad por defecto
           goal_poses.append(goal_pose)
           pose_names[i] = loc['name']

       # Iniciar navegación a través de los waypoints
       current_waypoint = 0
       total_waypoints = len(goal_poses)
       previous_pose = start_pose

       while current_waypoint < total_waypoints:
           # Navegar al waypoint actual
           goal_pose = goal_poses[current_waypoint]
           self.navigator.goToPose(goal_pose)
           self.publish_status("NAVIGATING", pose_names[current_waypoint])

           # Monitorear el progreso de la navegación
           while not self.navigator.isNavComplete():
               rclpy.spin_once(self.node, timeout_sec=1.0)
               feedback = self.navigator.getFeedback()
               if feedback:
                   current_pose = self.odom_subscriber.current_pose
                   if current_pose:
                       # Calcular y publicar el porcentaje de completado
                       completion_percentage = self.calculate_completion_percentage(
                           previous_pose, 
                           current_pose, 
                           {'x': goal_pose.pose.position.x, 'y': goal_pose.pose.position.y}
                       )
                       self.publish_status("NAVIGATING", pose_names[current_waypoint], completion_percentage)
                   
           # Procesar el resultado de la navegación
           result = self.navigator.getResult()
           if result == NavigationResult.SUCCEEDED:
               self.publish_status("REACHED", pose_names[current_waypoint])
               self.visited_waypoints.add(pose_names[current_waypoint])
               previous_pose = current_pose
               
           elif result == NavigationResult.CANCELED:
               self.publish_status("CANCELED", pose_names[current_waypoint])
               break
           elif result == NavigationResult.FAILED:
               self.publish_status("FAILED", pose_names[current_waypoint])
               break
               
           # Esperar confirmación para continuar al siguiente waypoint
           self.continue_subscriber.should_continue = False
           
           # Si no es el último waypoint, esperar confirmación para continuar
           if current_waypoint != total_waypoints - 1:
               self.publish_status("WAITING", pose_names[current_waypoint])
               while not self.continue_subscriber.should_continue:
                   rclpy.spin_once(self.node)
                   
           # Si es el último waypoint, ofrecer ir a la caja
           if current_waypoint == total_waypoints - 1:
               self.continue_subscriber.should_continue = True
               self.publish_status("FINISHED", pose_names[current_waypoint])
               while not self.todonext_subscriber.go_to_cashier:
                   rclpy.spin_once(self.node)
               if self.todonext_subscriber.go_to_cashier:
                   self.navigate_to_cashier()
                   break

           # Avanzar al siguiente waypoint
           current_waypoint += 1

       # Navegación completada
       self.publish_status("COMPLETED")
       return current_waypoint
       
   def navigate_to_cashier(self):
       
       #Navega directamente a la posición de la caja registradora
       #Función especial para el punto final predefinido
       
       # Preparar la pose objetivo para la caja registradora
       cash_pose = PoseStamped()
       cash_pose.header.frame_id = 'map'
       cash_pose.header.stamp = self.navigator.get_clock().now().to_msg()
       cash_pose.pose.position.x = self.fixed_cash_location['x']
       cash_pose.pose.position.y = self.fixed_cash_location['y']
       current_pose = self.odom_subscriber.current_pose
       
       # Calcular orientación apropiada
       if current_pose:
           yaw = self.calculate_goal_orientation(
               current_pose['x'], current_pose['y'], 
               self.fixed_cash_location['x'], self.fixed_cash_location['y']
           )
           q = quaternion_from_euler(0, 0, yaw)
           cash_pose.pose.orientation.x = q[0]
           cash_pose.pose.orientation.y = q[1]
           cash_pose.pose.orientation.z = q[2]
           cash_pose.pose.orientation.w = q[3]
       else:
           cash_pose.pose.orientation.w = 1.0  # Cuaternión identidad por defecto

       # Iniciar navegación a la caja
       self.navigator.goToPose(cash_pose)
       self.publish_status("NAVIGATING", "cashier")

       # Calcular distancia total para monitoreo de progreso
       initial_pose = self.odom_subscriber.current_pose
       goal_pose = {'x': cash_pose.pose.position.x, 'y': cash_pose.pose.position.y}
       total_distance = self.calculate_distance(initial_pose, goal_pose)

       # Monitorear el progreso de la navegación
       while not self.navigator.isNavComplete():
           rclpy.spin_once(self.node, timeout_sec=1.0)
           feedback = self.navigator.getFeedback()
           if feedback:
               current_pose = self.odom_subscriber.current_pose
               if current_pose:
                   remaining_distance = self.calculate_distance(current_pose, goal_pose)

                   # Calcular el porcentaje de completado
                   completion_percentage = ((total_distance - remaining_distance) / total_distance) * 100
                   completion_percentage = max(0, min(completion_percentage, 100))  

                   # Publicar el estado de navegación con el porcentaje de completado
                   self.publish_status("NAVIGATING", "cashier", completion_percentage)

       # Procesar el resultado de la navegación
       result = self.navigator.getResult()
       if result == NavigationResult.SUCCEEDED:
           self.publish_status("REACHED", "cashier")
       elif result == NavigationResult.CANCELED:
           self.publish_status("CANCELED", "cashier")
       elif result == NavigationResult.FAILED:
           self.publish_status("FAILED", "cashier")

   def calculate_goal_orientation(self, current_x, current_y, goal_x, goal_y):
   
       #Calcula la orientación óptima hacia el objetivo
       #Redondea a intervalos de 90 grados para movimientos más naturales
       
       dx = goal_x - current_x
       dy = goal_y - current_y
       angle = math.atan2(dy, dx)
       
       # Redondear al múltiplo más cercano de 90 grados (π/2 radianes)
       rounded_angle = round(angle / (math.pi / 2)) * (math.pi / 2)
       
       return rounded_angle


class OdomSubscriber:
   
   #Suscriptor a los mensajes de odometría
   #Mantiene actualizada la pose actual del robot
   
   def __init__(self, node):
       self.node = node
       self.subscription = node.create_subscription(
           Odometry,
           '/odom',
           self.odom_callback,
           10
       )
       self.current_pose = None

   def odom_callback(self, msg):
       self.current_pose = {
           'x': msg.pose.pose.position.x,
           'y': msg.pose.pose.position.y
       }

class ContinueSubscriber:
   
   #Suscriptor a mensajes de confirmación
   #Permite al usuario indicar cuándo continuar con la navegación
   
   def __init__(self, node):
       self.node = node
       self.subscription = node.create_subscription(
           String,
           '/continue_nav',
           self.continue_callback,
           10
       )
       self.should_continue = False

   def continue_callback(self, msg):
       if msg.data == "continue":
           self.should_continue = True


class ToDoNextSubscriber:
   
   #Suscriptor a mensajes de siguiente acción
   #Gestiona la decisión de ir a la caja registradora
   
   def __init__(self, node):
       self.node = node
       self.subscription = node.create_subscription(
           String,
           '/to_do_next',
           self.to_do_next_callback,
           10
       )
       self.go_to_cashier = False

   def to_do_next_callback(self, msg):
       self.received_message = msg.data
       if msg.data == "cash":
           self.go_to_cashier = True
       
def main(args=None):
   rclpy.init(args=args)
   navigator = AutonomousNavigator()
   navigator.navigate()
   rclpy.shutdown()

if __name__ == '__main__':
   main()