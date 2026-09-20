#!/usr/bin/env python3
# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Nodo de teleoperación mediante joystick para el robot SARA
# Implementa un nodo ROS2 que convierte los comandos de un control (gamepad)
# en mensajes de velocidad para controlar el movimiento del robot SARA.
# Permite el control manual directo, con modos de activación/desactivación
# y limitaciones de velocidad para garantizar la seguridad operativa.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class JoyTeleop(Node):
   def __init__(self):
       # Inicialización del nodo ROS
       # Configura suscripciones, publicadores y parámetros de control
       super().__init__('joy_teleop')
       self.subscription = self.create_subscription(
           Joy,
           'joy',
           self.joy_callback,
           10)
       self.publisher = self.create_publisher(Twist, 'cmd_vel_gamepad', 10)
       
       # Variables de control de velocidad
       # Limitaciones para garantizar una operación segura del robot SARA
       self.max_linear_speed = 0.22  # Velocidad máxima hacia adelante/atrás (m/s)
       self.max_angular_speed = 2.84 # Velocidad máxima de rotación (rad/s)
       
       # Variables para control de comunicación
       # Maneja el timeout para detener el robot si se pierde la señal del joystick
       self.timeout_duration = 0.1
       self.debounce_duration = 0.2
       self.last_received_time = self.get_clock().now()
       self.timer = self.create_timer(0.1, self.check_timeout)
       
       # Variables de control para el botón B
       # Gestiona la alternancia entre modos de operación (activo/inactivo)
       self.button_b_pressed = False
       self.counter = 0
       self.last_button_b_state = 0
       self.last_button_b_change_time = self.get_clock().now()
       self.mode_normal = False
       self.mode_special = True
       
       # Control de publicación
       # Gestiona la habilitación/deshabilitación de comandos y notificación de estado
       self.publish_enabled = False
       self.is_joy_on_publisher = self.create_publisher(String, 'is_joy_on', 10)
       self.is_joy_on_msg = String()

   def joy_callback(self, msg):
       # Manejo de mensajes del joystick
       # Procesa las señales del control y las convierte en comandos de movimiento
       current_time = self.get_clock().now()
       
       # Procesamiento del botón B para habilitar/deshabilitar la publicación
       # Implementa un mecanismo de alternancia con debouncing
       current_button_b_state = msg.buttons[1]
       time_since_last_change = (current_time - self.last_button_b_change_time).nanoseconds / 1e9
       
       if current_button_b_state != self.last_button_b_state and time_since_last_change > self.debounce_duration:
           self.last_button_b_change_time = current_time
           self.last_button_b_state = current_button_b_state

           if current_button_b_state == 1:
               if not self.button_b_pressed:
                   self.get_logger().info('Button B pressed')
                   self.button_b_pressed = True
                   self.counter += 1
                   if self.counter % 2 == 0:
                       self.get_logger().info('normal')
                       self.mode_normal = True
                       self.mode_special = False
                       self.publish_enabled = True
                   else:
                       self.get_logger().info('stopped')
                       self.mode_normal = False
                       self.mode_special = True
                       self.publish_enabled = False
           else:
               if self.button_b_pressed:
                   self.get_logger().info('Button B released')
                   self.button_b_pressed = False

       # Creación del mensaje Twist para comandos de velocidad
       # Define la velocidad lineal y angular según el estado del joystick
       twist = Twist()
       
       if self.mode_normal:
           # Modo normal: control activo del robot SARA mediante joystick
           self.is_joy_on_msg.data = 'yes'
           
           # Stick izquierdo para movimiento adelante/atrás (eje vertical)
           # Usa axes[1] para el movimiento vertical del stick izquierdo
           twist.linear.x = self.max_linear_speed * msg.axes[1]
           
           # Stick derecho para rotación (eje horizontal)
           # Usa axes[3] para el movimiento horizontal del stick derecho
           twist.angular.z = self.max_angular_speed * msg.axes[3]
           
           # El movimiento en Y se elimina para tracción diferencial
           twist.linear.y = 0.0

       elif self.mode_special:
           # Modo especial: detiene el movimiento del robot SARA
           twist.linear.x = 0.0
           twist.linear.y = 0.0
           twist.angular.z = 0.0
           self.publish_enabled = False
           self.is_joy_on_msg.data = 'no'

       # Publicación del estado del joystick
       self.is_joy_on_publisher.publish(self.is_joy_on_msg)
       
       # Publicación del mensaje Twist solo si está habilitado
       # Envía los comandos de velocidad cuando el control está activo
       if self.publish_enabled:
           self.publisher.publish(twist)

       self.last_received_time = self.get_clock().now()

   def check_timeout(self):
       # Verificación de timeout para seguridad
       # Detiene el robot si no se reciben mensajes del joystick durante el tiempo configurado
       current_time = self.get_clock().now()
       time_since_last_msg = (current_time - self.last_received_time).nanoseconds / 1e9

       if time_since_last_msg > self.timeout_duration:
           if self.mode_normal:
               # Envía comando de velocidad cero si se supera el timeout
               twist = Twist()
               if self.publish_enabled:
                   self.publisher.publish(twist)

def main(args=None):
   # Función principal del programa
   # Inicializa y ejecuta el nodo de teleoperación
   rclpy.init(args=args)
   node = JoyTeleop()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()