# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Archivo de lanzamiento para nodos básicos de control del robot SARA
# Implementa la configuración de lanzamiento para los componentes fundamentales
# de control del robot, incluyendo teleoperación mediante joystick y un sistema
# básico de evasión de obstáculos. Este archivo proporciona las funcionalidades
# esenciales para el control manual y asistido del robot en operación.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
 
    # Configuración del tiempo de ejecución
    # Define si se utiliza el tiempo de simulación o el tiempo real
    use_sim_time = LaunchConfiguration('use_sim_time')
   
    # Nodo de evasión básica de obstáculos
    # Implementa un algoritmo simple para detectar y evitar colisiones
    # mientras el robot SARA está en movimiento
    obstacle_node = Node(
            package='sara_safety',
            executable='naive_obstacle_avoidance.py'
         )

    # Nodo de teleoperación con joystick
    # Permite el control manual del robot SARA mediante un joystick,
    # traduciendo las señales del control a comandos de velocidad
    joy_teleop_node = Node(
            package='sara_safety',
            executable='joy_teleop.py'
         )
    
    # Descripción de lanzamiento 
    # Configura la secuencia de inicio de los componentes básicos de control
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        obstacle_node,
        joy_teleop_node,

    ])