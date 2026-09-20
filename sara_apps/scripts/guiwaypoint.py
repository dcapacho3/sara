#!/usr/bin/env python3
# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Sistema de navegación autónoma para el robot SARA en entorno simulado
# Implementa un algoritmo de planificación de rutas que optimiza el recorrido entre
# productos seleccionados, utilizando algoritmos de optimización de trayectorias
# para encontrar la ruta más eficiente durante el proceso de compra.

import sqlite3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String  # Importa el mensaje de tipo String para el control
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
import numpy as np
import yaml
from PIL import Image
import os
from itertools import permutations
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_from_euler

def get_source_db_path(package_name, db_filename):
    # Función para obtener la ruta de la base de datos en el directorio src del paquete
    # Navega desde el directorio share hasta la ubicación de la base de datos
    # siguiendo la estructura estándar de un workspace ROS2

    # Obtener el directorio share del paquete
    share_dir = get_package_share_directory(package_name)
    
    # Navegar hasta la raíz del workspace (subir 4 niveles: share/package/install/workspace)
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(share_dir))))
    
    # Construir la ruta a la base de datos en src
    db_path = os.path.join(workspace_root, 'src', 'sara', package_name, 'database', db_filename)
    
    #print(f"Trying to access database at: {db_path}")
    
    return db_path

class AutonomousNavigator:
    # Clase principal que implementa el navegador autónomo
    # Gestiona la navegación del robot entre los productos seleccionados
    # y optimiza la ruta para minimizar la distancia total recorrida
    def __init__(self, navigator=None):
        # Inicialización del navegador autónomo
        # Configura el nodo, suscriptores, publicadores y variables de estado
        self.node = rclpy.create_node('navigator_node')
        self.odom_subscriber = OdomSubscriber(self.node)
        self.continue_subscriber = ContinueSubscriber(self.node)
        self.todonext_subscriber = ToDoNextSubscriber(self.node)
        self.status_publisher = self.node.create_publisher(String, '/navigation_status', 10)
        self.visited_waypoints = set()

        # Si el llamador ya tiene un BasicNavigator (base_navgui.py crea uno
        # en init_ros para esperar a que Nav2 esté activo), se reutiliza en
        # vez de crear otro: dos instancias con el mismo nombre de nodo
        # ('basic_navigator') coexistiendo en el mismo proceso disparaban el
        # warning "Publisher already registered for node name: basic_navigator"
        # y duplicaban la espera/los clientes de acción contra Nav2, lo cual
        # era una causa real de que Nav2 pareciera no arrancar a veces. Al
        # ejecutarse como script independiente (__main__ más abajo) no se
        # pasa nada, así que se sigue creando uno propio.
        if navigator is not None:
            self.navigator = navigator
        else:
            self.navigator = BasicNavigator()
            self.navigator.waitUntilNav2Active()
        
        self.fixed_cash_location = {'name': 'cashier', 'x': -1.0, 'y': -2.0}

    def publish_status(self, status, waypoint_name=None, completion_percentage=None):
        # Método para publicar el estado actual de la navegación
        # Envía información sobre el estado, waypoint actual y porcentaje de completado
        msg = String()
        msg.data = f"{status}|{waypoint_name or ''}|{completion_percentage or ''}|{','.join(self.visited_waypoints)}"
        self.status_publisher.publish(msg)

    def get_product_locations(self):
        # Método para obtener las ubicaciones de los productos desde la base de datos
        # Recupera las coordenadas de los productos seleccionados y añade la ubicación de la caja
        db_dir = get_source_db_path('sara_apps', 'products.db')
        conn = sqlite3.connect(db_dir)
        cursor = conn.cursor()
        cursor.execute('SELECT name, x, y FROM selected_products')
        locations = [{'name': row[0], 'x': row[1], 'y': row[2]} for row in cursor.fetchall()]
        
        conn.close()
        locations.append(self.fixed_cash_location)
     
        return locations

    def calculate_distance(self, p1, p2):
        # Método para calcular la distancia euclidiana entre dos puntos
        # Utiliza la fórmula de distancia entre dos puntos en un plano cartesiano
        return math.sqrt((p1['x'] - p2['x'])**2 + (p1['y'] - p2['y'])**2)

    def nearest_neighbor_with_fixed_end(self, start, locations, end):
        # Método que implementa el algoritmo del vecino más cercano con punto final fijo
        # Construye una ruta comenzando en start, visitando todos los puntos en locations
        # y terminando en end (la caja)
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
        # Método que implementa un intercambio 2-opt para optimización de rutas
        # Invierte el orden de los puntos entre las posiciones i y j en el camino
        return path[:i] + path[i:j+1][::-1] + path[j+1:]

    def two_opt_improvement(self, path):
        # Método que aplica mejoras usando el algoritmo 2-opt
        # Realiza intercambios 2-opt hasta que no haya mejora posible en la distancia total
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
        # Método para calcular la distancia total de una ruta
        # Suma las distancias entre puntos consecutivos en la ruta
        return sum(self.calculate_distance(path[i], path[i+1]) for i in range(len(path) - 1))
        
    def optimize_path(self, start, locations, end):
        # Método para optimizar la ruta entre las ubicaciones
        # Aplica primero el algoritmo del vecino más cercano y luego mejoras 2-opt
        initial_path = self.nearest_neighbor_with_fixed_end(start, locations, end)
        optimized_path = self.two_opt_improvement(initial_path)
        return optimized_path

    def calculate_completion_percentage(self, previous_pose, current_pose, goal_pose):
        # Método para calcular el porcentaje de completado de la navegación actual
        # Basado en la distancia recorrida respecto a la distancia total
        total_distance = self.calculate_distance(previous_pose, goal_pose)
        remaining_distance = self.calculate_distance(current_pose, goal_pose)
        completion_percentage = ((total_distance - remaining_distance) / total_distance) * 100
        return max(0, min(completion_percentage, 100))

    def navigate(self):
        # Método principal que implementa la navegación completa
        # Coordina todo el proceso de navegación entre productos
        print("Waiting for initial position from odometry...")
        while self.odom_subscriber.current_pose is None:
            rclpy.spin_once(self.node)
        start_pose = self.odom_subscriber.current_pose
        print(f"Initial position: {start_pose}")

        # Obtener y verificar ubicaciones
        locations = self.get_product_locations()
        if not locations:
            print("No selected products found.")
            return

        # Identificar posición de caja y otros productos
        cashier = next(loc for loc in locations if loc['name'] == 'cashier')
        other_locations = [loc for loc in locations if loc['name'] != 'cashier']

        # Optimizar la ruta
        optimized_path = self.optimize_path(start_pose, other_locations, cashier)

        # Solicitar confirmación del usuario antes de iniciar la navegación
        self.publish_status("READY")
        while not self.continue_subscriber.should_continue:
            rclpy.spin_once(self.node)
        
        # Crear las pose goals para la navegación
        goal_poses = []
        pose_names = {}

        for i, loc in enumerate(optimized_path[1:-1]):  # Skip start_pose
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = loc['x']
            goal_pose.pose.position.y = loc['y']
            current_pose = self.odom_subscriber.current_pose
            if current_pose:
                yaw = self.calculate_goal_orientation(current_pose['x'], current_pose['y'], loc['x'], loc['y'])
                q = quaternion_from_euler(0, 0, yaw)
                goal_pose.pose.orientation.x = q[0]
                goal_pose.pose.orientation.y = q[1]
                goal_pose.pose.orientation.z = q[2]
                goal_pose.pose.orientation.w = q[3]
            else:
                goal_pose.pose.orientation.w = 1.0  # Default to identity quaternion if no current pose
            goal_poses.append(goal_pose)
            pose_names[i] = loc['name']

        # Iniciar la navegación hacia cada waypoint
        current_waypoint = 0
        total_waypoints = len(goal_poses)
        previous_pose = start_pose

        while current_waypoint < total_waypoints:
            # Navegar hacia el waypoint actual
            goal_pose = goal_poses[current_waypoint]
            self.navigator.goToPose(goal_pose)
            self.publish_status("NAVIGATING", pose_names[current_waypoint])

            # Monitorear el progreso de la navegación
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self.node, timeout_sec=1.0)
                feedback = self.navigator.getFeedback()
                if feedback:
                    current_pose = self.odom_subscriber.current_pose
                    if current_pose:
                        completion_percentage = self.calculate_completion_percentage(previous_pose, current_pose, {'x': goal_pose.pose.position.x, 'y': goal_pose.pose.position.y})
                        self.publish_status("NAVIGATING", pose_names[current_waypoint], completion_percentage)
                    
            # Procesar el resultado de la navegación
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.publish_status("REACHED", pose_names[current_waypoint])
                self.visited_waypoints.add(pose_names[current_waypoint])
                previous_pose = current_pose
                
            elif result == TaskResult.CANCELED:
                self.publish_status("CANCELED", pose_names[current_waypoint])
                break
            elif result == TaskResult.FAILED:
                self.publish_status("FAILED", pose_names[current_waypoint])
                break
            
            # Esperar confirmación para continuar al siguiente waypoint
            self.continue_subscriber.should_continue = False
            
            if current_waypoint != total_waypoints -1 :
                self.publish_status("WAITING", pose_names[current_waypoint])
       
                while not self.continue_subscriber.should_continue:
                    rclpy.spin_once(self.node)
                    
            # Manejar el final de la navegación
            if current_waypoint == total_waypoints - 1:
                self.continue_subscriber.should_continue = True
                self.publish_status("FINISHED", pose_names[current_waypoint])
                while not self.todonext_subscriber.go_to_cashier and not self.todonext_subscriber.shop_again:
                    rclpy.spin_once(self.node)
                if self.todonext_subscriber.go_to_cashier:
                    self.navigate_to_cashier()
                    break
                elif self.todonext_subscriber.shop_again:
                    self.publish_status("SHOPPING_AGAIN")
                    break
         
            current_waypoint += 1

        self.publish_status("COMPLETED")

        return current_waypoint
        
    def navigate_to_cashier(self):
        # Método para navegar hacia la posición de la caja
        # Configura y ejecuta la navegación final hacia la caja
        cash_pose = PoseStamped()
        cash_pose.header.frame_id = 'map'
        cash_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        cash_pose.pose.position.x = self.fixed_cash_location['x']
        cash_pose.pose.position.y = self.fixed_cash_location['y']
        current_pose = self.odom_subscriber.current_pose
        if current_pose:
            yaw = self.calculate_goal_orientation(current_pose['x'], current_pose['y'], 
                                                  self.fixed_cash_location['x'], self.fixed_cash_location['y'])
            q = quaternion_from_euler(0, 0, yaw)
            cash_pose.pose.orientation.x = q[0]
            cash_pose.pose.orientation.y = q[1]
            cash_pose.pose.orientation.z = q[2]
            cash_pose.pose.orientation.w = q[3]
        else:
            cash_pose.pose.orientation.w = 1.0  # Default to identity quaternion if no current pose

        self.navigator.goToPose(cash_pose)
        self.publish_status("NAVIGATING", "cashier")

        # Cálculo de la distancia inicial para el seguimiento del progreso
        initial_pose = self.odom_subscriber.current_pose
        goal_pose = {'x': cash_pose.pose.position.x, 'y': cash_pose.pose.position.y}

        # Calcular la distancia total desde la posición inicial hasta el destino
        total_distance = self.calculate_distance(initial_pose, goal_pose)
 
        # Monitorear el progreso de la navegación hacia la caja
        while not self.navigator.isTaskComplete():
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

        # Procesar el resultado de la navegación a la caja
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.publish_status("REACHED", "cashier")
        elif result == TaskResult.CANCELED:
            self.publish_status("CANCELED", "cashier")
        elif result == TaskResult.FAILED:
            self.publish_status("FAILED", "cashier")

    def calculate_goal_orientation(self, current_x, current_y, goal_x, goal_y):
        # Método para calcular la orientación objetivo
        # Determina la dirección en la que el robot debe mirar al llegar al destino
        dx = goal_x - current_x
        dy = goal_y - current_y
        angle = math.atan2(dy, dx)
        
        # Round to the nearest 90 degrees (π/2 radians)
        rounded_angle = round(angle / (math.pi / 2)) * (math.pi / 2)
        
        return rounded_angle


class OdomSubscriber:
    # Clase que implementa un suscriptor para mensajes de odometría
    # Permite obtener y actualizar la posición actual del robot
    def __init__(self, node):
        # Inicialización del suscriptor de odometría
        # Configura la suscripción al tópico de odometría
        self.node = node
        self.subscription = node.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.current_pose = None

    def odom_callback(self, msg):
        # Callback para procesar mensajes de odometría
        # Extrae la posición x,y del robot del mensaje recibido
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        }

class ContinueSubscriber:
    # Clase que implementa un suscriptor para comandos de continuación
    # Permite que la interfaz de usuario indique cuándo continuar la navegación
    def __init__(self, node):
        # Inicialización del suscriptor de continuación
        # Configura la suscripción al tópico de comandos de continuación
        self.node = node
        self.subscription = node.create_subscription(
            String,
            '/continue_nav',
            self.continue_callback,
            10
        )
        self.should_continue = False

    def continue_callback(self, msg):
        # Callback para procesar comandos de continuación
        # Actualiza el estado según el mensaje recibido
        if msg.data == "continue":
            self.should_continue = True


class ToDoNextSubscriber:
    # Clase que implementa un suscriptor para comandos de acción siguiente
    # Permite determinar si se debe ir a la caja o volver a comprar
    def __init__(self, node):
        # Inicialización del suscriptor de acción siguiente
        # Configura la suscripción al tópico de comandos de acción
        self.node = node
        self.subscription = node.create_subscription(
            String,
            '/to_do_next',
            self.to_do_next_callback,
            10
        )
        self.go_to_cashier = False
        self.shop_again = False

    def to_do_next_callback(self, msg):
        # Callback para procesar comandos de acción siguiente
        # Actualiza los estados según el mensaje recibido
        self.received_message = msg.data
        if msg.data == "cash":
            self.go_to_cashier = True
        elif msg.data == "shop_again":
            self.shop_again = True
        
        
def main(args=None):
    # Función principal del programa
    # Inicializa ROS2, crea el navegador y ejecuta la navegación
    rclpy.init(args=args)
    navigator = AutonomousNavigator()
    navigator.navigate()
    rclpy.shutdown()

if __name__ == '__main__':
    main()