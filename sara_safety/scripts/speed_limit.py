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
from std_msgs.msg import Float32

class TwistLimiter(Node):
    def __init__(self):
        # Inicialización del nodo ROS
        # Configura suscriptor, publicador y límites de velocidad
        super().__init__('twist_limiter')

        # Define velocidades máximas lineales y angulares como variables
        # Estos valores determinan los límites superiores de movimiento del robot
        self.MAX_LINEAR_SPEED = 0.15
        self.MAX_ANGULAR_SPEED = 1.82

        # Factor de escala por peso de carga (weight_monitor.py, cart_speed_scale).
        # 1.0 = sin restricción; se reduce hacia 0.0 según la carga detectada
        # en la báscula del carrito. Empieza en 1.0 (sin restricción) hasta
        # que llegue la primera lectura real.
        self.speed_scale = 1.0

        # Límite de velocidad continuo de Adaptive Separation
        # (adaptive_separation.py, docs/paper2_draft.tex sec 3.2/4.1) - se
        # compone con MAX_LINEAR_SPEED vía min(), NO como otro tópico de
        # twist_mux: es un límite continuo dependiente de distancia, no un
        # veto binario como Proximity Stop, así que encaja en la misma capa
        # que el escalado por peso de Payload Interaction. Empieza en
        # MAX_LINEAR_SPEED (sin restricción) hasta la primera lectura real,
        # igual que speed_scale arriba - adaptive_separation.py también
        # publica ese mismo valor (v_max_platform) cuando no hay nada en su
        # rango válido, así que este arranque coincide con su propio estado
        # de "zona despejada", no es una suposición separada.
        self.adaptive_sep_v_max = self.MAX_LINEAR_SPEED

        # Crea suscripción para recibir comandos de velocidad
        # Se suscribe al tópico 'cmd_vel_in' para recibir mensajes Twist
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_in',
            self.listener_callback,
            10)

        # Se suscribe al factor de escala publicado por weight_monitor.py
        self.scale_subscription = self.create_subscription(
            Float32,
            'cart_speed_scale',
            self.scale_callback,
            10)

        # Se suscribe al límite de velocidad publicado por adaptive_separation.py
        self.adaptive_sep_subscription = self.create_subscription(
            Float32,
            'adaptive_separation_v_max_mps',
            self.adaptive_sep_callback,
            10)

        # Crea publicador para enviar comandos de velocidad limitados
        # Publica en el tópico 'cmd_vel_out' mensajes Twist modificados
        self.publisher = self.create_publisher(Twist, 'cmd_vel_out', 10)

    def scale_callback(self, msg):
        self.speed_scale = max(0.0, min(1.0, msg.data))

    def adaptive_sep_callback(self, msg):
        self.adaptive_sep_v_max = max(0.0, msg.data)

    def listener_callback(self, msg):
        # Método para procesar los mensajes de velocidad recibidos
        # Crea un nuevo mensaje con velocidades limitadas según los máximos definidos
        limited_msg = Twist()

        # Límites efectivos: el menor entre el máximo escalado por carga
        # (Payload Interaction, self.speed_scale) y el límite continuo de
        # Adaptive Separation (self.adaptive_sep_v_max) - min(), no un
        # tercer factor multiplicativo, porque Adaptive Separation ya
        # calcula un límite absoluto en m/s (Ecuación 2 del paper), no un
        # factor 0-1 que tendría que combinarse por escala. El límite
        # angular solo lo escala la carga, Adaptive Separation no tiene
        # opinión sobre giro en el sitio.
        max_linear = min(self.MAX_LINEAR_SPEED * self.speed_scale, self.adaptive_sep_v_max)
        max_angular = self.MAX_ANGULAR_SPEED * self.speed_scale

        # Limita velocidad lineal usando max_linear
        # Aplica restricciones en los tres ejes (x, y, z)
        limited_msg.linear.x = max(min(msg.linear.x, max_linear), -max_linear)
        limited_msg.linear.y = max(min(msg.linear.y, max_linear), -max_linear)
        limited_msg.linear.z = max(min(msg.linear.z, max_linear), -max_linear)

        # Limita velocidad angular usando max_angular
        # Aplica restricciones en los tres ejes de rotación (x, y, z)
        limited_msg.angular.x = max(min(msg.angular.x, max_angular), -max_angular)
        limited_msg.angular.y = max(min(msg.angular.y, max_angular), -max_angular)
        limited_msg.angular.z = max(min(msg.angular.z, max_angular), -max_angular)

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