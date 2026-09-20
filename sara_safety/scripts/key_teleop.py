#!/usr/bin/env python3

# Descripción: Nodo de teleoperación mediante teclado para el robot SARA
# Implementa un nodo ROS2 que convierte comandos de teclado en mensajes de velocidad
# para controlar el movimiento del robot SARA. Permite el control manual directo
# mediante las teclas WASD y espacio, aplicando limitaciones de velocidad y
# proporcionando información de retroalimentación al operador.

import os
import select
import sys
import rclpy

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

if os.name == 'nt':
   import msvcrt
else:
   import termios
   import tty

# Configuración de límites de velocidad
# Define valores máximos para garantizar la operación segura del robot SARA
MAX_LINEAR_VELOCITY = 0.22  # Velocidad lineal máxima (m/s)
MAX_ANGULAR_VELOCITY = 2.84  # Velocidad angular máxima (rad/s)

# Incrementos de velocidad por pulsación de tecla
# Controla la sensibilidad de respuesta a los comandos del teclado
LIN_VEL_STEP_SIZE = 0.01  # Incremento de velocidad lineal por pulsación
ANG_VEL_STEP_SIZE = 0.1   # Incremento de velocidad angular por pulsación

# Mensaje de instrucciones para el usuario
# Proporciona guía sobre el control del robot mediante teclado
msg = """
Control Your TurtleBot3 Burger!
---------------------------
Moving around:
       w
  a    s    d
       x

w/x : increase/decrease linear velocity (~ 0.22)
a/d : increase/decrease angular velocity (~ 2.84)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


def get_key(settings):
   # Función para capturar teclas presionadas
   # Detecta la entrada del teclado de manera no bloqueante
   if os.name == 'nt':
       return msvcrt.getch().decode('utf-8')
   tty.setraw(sys.stdin.fileno())
   rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
   if rlist:
       key = sys.stdin.read(1)
   else:
       key = ''

   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   return key


def print_vels(target_linear_velocity, target_angular_velocity):
   # Función para mostrar las velocidades actuales
   # Proporciona retroalimentación visual al operador
   print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
       target_linear_velocity,
       target_angular_velocity))


def make_simple_profile(output, input, slop):
   # Función para suavizar cambios de velocidad
   # Evita cambios bruscos que podrían desestabilizar el robot SARA
   if input > output:
       output = min(input, output + slop)
   elif input < output:
       output = max(input, output - slop)
   else:
       output = input

   return output


def constrain(input_vel, low_bound, high_bound):
   # Función para limitar valores dentro de un rango
   # Asegura que las velocidades no excedan los límites definidos
   if input_vel < low_bound:
       input_vel = low_bound
   elif input_vel > high_bound:
       input_vel = high_bound
   else:
       input_vel = input_vel

   return input_vel


def check_linear_limit_velocity(velocity):
   # Función para verificar los límites de velocidad lineal
   # Aplica restricciones para movimiento seguro hacia adelante/atrás
   return constrain(velocity, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY)


def check_angular_limit_velocity(velocity):
   # Función para verificar los límites de velocidad angular
   # Aplica restricciones para rotación segura
   return constrain(velocity, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)


def main():
   # Función principal del programa
   # Inicializa y ejecuta el nodo de teleoperación por teclado
   settings = None
   if os.name != 'nt':
       settings = termios.tcgetattr(sys.stdin)

   # Inicialización del nodo ROS
   rclpy.init()

   # Configuración del publicador para comandos de velocidad
   qos = QoSProfile(depth=10)
   node = rclpy.create_node('teleop_keyboard')
   pub = node.create_publisher(Twist, 'cmd_vel_key', qos)

   # Variables de control para velocidades y estado
   status = 0
   target_linear_velocity = 0.0
   target_angular_velocity = 0.0
   control_linear_velocity = 0.0
   control_angular_velocity = 0.0

   try:
       # Bucle principal de control
       # Procesa las teclas y genera los comandos de velocidad correspondientes
       print(msg)
       while(1):
           # Captura de tecla presionada
           key = get_key(settings)
           
           # Interpretación del comando según la tecla
           if key == 'w':  # Incrementar velocidad hacia adelante
               target_linear_velocity =\
                   check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
               status = status + 1
               print_vels(target_linear_velocity, target_angular_velocity)
           elif key == 'x':  # Incrementar velocidad hacia atrás
               target_linear_velocity =\
                   check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
               status = status + 1
               print_vels(target_linear_velocity, target_angular_velocity)
           elif key == 'a':  # Incrementar velocidad de giro a la izquierda
               target_angular_velocity =\
                   check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
               status = status + 1
               print_vels(target_linear_velocity, target_angular_velocity)
           elif key == 'd':  # Incrementar velocidad de giro a la derecha
               target_angular_velocity =\
                   check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
               status = status + 1
               print_vels(target_linear_velocity, target_angular_velocity)
           elif key == ' ' or key == 's':  # Detener completamente el robot
               target_linear_velocity = 0.0
               control_linear_velocity = 0.0
               target_angular_velocity = 0.0
               control_angular_velocity = 0.0
               print_vels(target_linear_velocity, target_angular_velocity)
           else:
               if (key == '\x03'):  # Ctrl+C para salir
                   break

           # Mostrar instrucciones periódicamente
           if status == 20:
               print(msg)
               status = 0

           # Creación del mensaje Twist para comandos de velocidad
           twist = Twist()

           # Suavizado de la velocidad lineal
           control_linear_velocity = make_simple_profile(
               control_linear_velocity,
               target_linear_velocity,
               (LIN_VEL_STEP_SIZE / 2.0))

           # Asignación de velocidad lineal (solo en eje X para robot diferencial)
           twist.linear.x = control_linear_velocity
           twist.linear.y = 0.0
           twist.linear.z = 0.0

           # Suavizado de la velocidad angular
           control_angular_velocity = make_simple_profile(
               control_angular_velocity,
               target_angular_velocity,
               (ANG_VEL_STEP_SIZE / 2.0))

           # Asignación de velocidad angular (solo en eje Z para rotación en plano)
           twist.angular.x = 0.0
           twist.angular.y = 0.0
           twist.angular.z = control_angular_velocity

           # Publicación del comando de velocidad
           pub.publish(twist)

   except Exception as e:
       # Manejo de excepciones
       print(e)

   finally:
       # Asegurar detención del robot al finalizar
       # Envía comandos de velocidad cero para una parada segura
       twist = Twist()
       twist.linear.x = 0.0
       twist.linear.y = 0.0
       twist.linear.z = 0.0

       twist.angular.x = 0.0
       twist.angular.y = 0.0
       twist.angular.z = 0.0

       pub.publish(twist)

       # Restaurar configuración de terminal
       if os.name != 'nt':
           termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
   main()