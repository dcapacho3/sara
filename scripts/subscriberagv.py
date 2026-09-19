#!/usr/bin/env python3
# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Suscriptor múltiple de tópicos para robot SARA
# Implementa un nodo que se suscribe a diversos tópicos de ROS2 para
# monitorear el estado del robot, incluyendo odometría, escaneo láser,
# transformaciones, estados de articulaciones y mensajes personalizados.
# Permite la captura y procesamiento de datos de sensores para diagnóstico
# y desarrollo.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState, PointCloud2
from tf2_msgs.msg import TFMessage
import numpy as np
from std_msgs.msg import String

class MySubscriber(Node):

    def __init__(self):
        # Inicialización del nodo ROS
        # Configura todas las suscripciones a diferentes tópicos
        super().__init__('my_subscriber')

        # Suscripción al tópico de Odometría
        # Recibe datos de posición y orientación del robot desde el odómetro
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        # Suscripción al tópico de Odometría filtrada
        # Recibe datos de posición y orientación procesados con filtros
        self.odom_filtered_subscription = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.odom_real_callback,
            10)
        
        # Suscripción al tópico de escaneo láser
        # Recibe datos de distancia del sensor LIDAR
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            100)

        # Suscripción al tópico de transformaciones TF
        # Recibe datos de relaciones espaciales entre marcos de coordenadas
        self.tf_subscription = self.create_subscription(
            TFMessage,
            'tf',
            self.tf_callback,
            10)

        # Suscripción al tópico de transformaciones TF estáticas
        # Recibe datos de relaciones espaciales fijas entre marcos de coordenadas
        self.tf_static_subscription = self.create_subscription(
            TFMessage,
            'tf_static',
            self.tf_static_callback,
            10)

        # Suscripción al tópico de estados de articulaciones
        # Recibe datos de posición y velocidad de las articulaciones del robot
        self.joint_states_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10)
            
        # Suscripción al tópico de mensajes personalizados
        # Recibe mensajes de texto para diagnóstico o control
        self.message_py_subscription = self.create_subscription(
            String,
            'message_py',
            self.message_py_callback,
            10)

        # Suscripción al tópico de nube de puntos (comentado)
        # Código desactivado para recibir datos 3D del entorno
        #self.point_cloud_subscription = self.create_subscription(
        #    PointCloud2,
        #    'point_cloud',
        #    self.point_cloud_callback,
        #    10)

        # Referencias a las suscripciones para evitar advertencias de variables no utilizadas
        # Previene que el recolector de basura elimine las suscripciones
        self.odom_subscription  # prevent unused variable warning
        self.odom_filtered_subscription  # prevent unused variable warning
        self.scan_subscription  # prevent unused variable warning
        self.tf_subscription  # prevent unused variable warning
        self.tf_static_subscription

        #self.point_cloud_subscription 

    def odom_callback(self, msg):
        # Método para procesar mensajes de odometría
        # Extrae información de posición y orientación del robot
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        #self.get_logger().info(f'Odometry - Position: [{position.x}, {position.y}]')
        #self.get_logger().info(f'Odometry - Orientation: [{np.rad2deg(orientation.z)}]')
    
    def odom_real_callback(self, msg):
        # Método para procesar mensajes de odometría filtrada
        # Extrae y registra información de posición real del robot
        position_filtered = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.get_logger().info(f'Odometry Real - Position: [{position_filtered.x}, {position_filtered.y}]')
        #self.get_logger().info(f'Odometry - Orientation: [{np.rad2deg(orientation.z)}]')

    def scan_callback(self, msg):
        # Método para procesar mensajes de escaneo láser
        # Almacena datos de distancias detectadas por el sensor LIDAR
        self.scan_ranges = msg.ranges
        leng = len(self.scan_ranges)
        #print("ScanLength: ", leng)
        #print("Scan: ", self.scan_ranges)
        #pass

    def tf_callback(self, msg):
        # Método para procesar mensajes de transformaciones
        # Maneja datos de relaciones espaciales entre marcos de coordenadas
        # for transform in msg.transforms:
        #self.get_logger().info(f'TF - Transform: {transform}')
        pass

    def tf_static_callback(self, msg):
        # Método para procesar mensajes de transformaciones estáticas
        # Maneja datos de relaciones espaciales fijas
        # for transform in msg.transforms:
        #self.get_logger().info(f'TF Static - Transform: {transform}')
        pass

    def voltage_callback(self, msg):
        # Método para procesar mensajes de voltaje
        # Monitorea el nivel de batería del robot
        #self.get_logger().info(f'Voltage: {msg.data}')
        pass

    def joint_states_callback(self, msg):
        # Método para procesar mensajes de estados de articulaciones
        # Maneja datos de posición y velocidad de las articulaciones
        # self.get_logger().info(f'Joint States: {msg}')
        pass

    def point_cloud_callback(self, msg):
        # Método para procesar mensajes de nube de puntos
        # Maneja datos 3D del entorno capturados por sensores
        #self.get_logger().info('Processing Point Cloud message')
        pass
        
    def message_py_callback(self, msg):
        # Método para procesar mensajes personalizados
        # Maneja cadenas de texto para diagnóstico o control
        #self.get_logger().info(f'Message Py: {msg.data}')
        pass

def main(args=None):
    # Función principal del programa
    # Inicializa el nodo y lo mantiene en ejecución hasta la interrupción
    
    # Inicialización de la infraestructura ROS
    rclpy.init(args=args)
    
    # Crea e inicia el nodo suscriptor
    subscriber = MySubscriber()
    
    # Mantiene el nodo activo hasta recibir señal de terminación
    rclpy.spin(subscriber)
    
    # Limpieza final de recursos
    # Asegura que el nodo se destruya correctamente
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()