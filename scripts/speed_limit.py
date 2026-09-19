#!/usr/bin/env python3
# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Limitador de velocidad para robot SARA
# Implementa un nodo que restringe las velocidades lineales y angulares
# recibidas en mensajes Twist. Esto previene movimientos bruscos o demasiado
# rápidos que podrían comprometer la seguridad del robot o afectar
# la precisión de su navegación.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistLimiter(Node):
    def __init__(self):
        # Inicialización del nodo ROS
        # Configura suscriptor, publicador y límites de velocidad
        super().__init__('twist_limiter')
        
        # Define velocidades máximas lineales y angulares como variables
        # Estos valores determinan los límites superiores de movimiento del robot
        self.MAX_LINEAR_SPEED = 0.15
        self.MAX_ANGULAR_SPEED = 1.82
        
        # Crea suscripción para recibir comandos de velocidad
        # Se suscribe al tópico 'cmd_vel_in' para recibir mensajes Twist
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_in',
            self.listener_callback,
            10)
            
        # Crea publicador para enviar comandos de velocidad limitados
        # Publica en el tópico 'cmd_vel_out' mensajes Twist modificados
        self.publisher = self.create_publisher(Twist, 'cmd_vel_out', 10)

    def listener_callback(self, msg):
        # Método para procesar los mensajes de velocidad recibidos
        # Crea un nuevo mensaje con velocidades limitadas según los máximos definidos
        limited_msg = Twist()
        
        # Limita velocidad lineal usando MAX_LINEAR_SPEED
        # Aplica restricciones en los tres ejes (x, y, z)
        limited_msg.linear.x = max(min(msg.linear.x, self.MAX_LINEAR_SPEED), -self.MAX_LINEAR_SPEED)
        limited_msg.linear.y = max(min(msg.linear.y, self.MAX_LINEAR_SPEED), -self.MAX_LINEAR_SPEED)
        limited_msg.linear.z = max(min(msg.linear.z, self.MAX_LINEAR_SPEED), -self.MAX_LINEAR_SPEED)
        
        # Limita velocidad angular usando MAX_ANGULAR_SPEED
        # Aplica restricciones en los tres ejes de rotación (x, y, z)
        limited_msg.angular.x = max(min(msg.angular.x, self.MAX_ANGULAR_SPEED), -self.MAX_ANGULAR_SPEED)
        limited_msg.angular.y = max(min(msg.angular.y, self.MAX_ANGULAR_SPEED), -self.MAX_ANGULAR_SPEED)
        limited_msg.angular.z = max(min(msg.angular.z, self.MAX_ANGULAR_SPEED), -self.MAX_ANGULAR_SPEED)
        
        # Publica el mensaje con velocidades limitadas
        self.publisher.publish(limited_msg)

def main(args=None):
    # Función principal del programa
    # Inicializa el nodo y lo mantiene en ejecución hasta la interrupción
    
    # Inicialización de la infraestructura ROS
    rclpy.init(args=args)
    
    # Crea e inicia el nodo limitador
    twist_limiter = TwistLimiter()
    
    # Mantiene el nodo activo hasta recibir señal de terminación
    rclpy.spin(twist_limiter)
    
    # Limpieza final de recursos
    # Asegura que el nodo se destruya correctamente
    twist_limiter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()