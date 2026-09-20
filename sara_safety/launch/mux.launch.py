# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Archivo de lanzamiento para multiplexor de tópicos

# Implementa la configuración de lanzamiento para un sistema
# de control de movimiento basado en multiplexación de tópicos de velocidad. 
# La arquitectura permite la integración de múltiples fuentes de comandos con 
# prioridades definidas, facilitando la conmutación entre control manual y 
# algoritmos autónomos de navegación, evasion de obstaculos y bloqueo.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declaración del argumento para el tópico de salida de comandos de velocidad
    # Permite la configuración del destino de los mensajes generados por el multiplexor

    # Declaración del argumento para el tópico de salida de comandos de velocidad
    DeclareLaunchArgument(
            'cmd_vel_out',
            default_value='/cmd_vel_out',
            description='cmd vel output topic'),

    # Configuración del tiempo de ejecucion
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Carga del archivo de configuración para el multiplexor
    mux_params = os.path.join(get_package_share_directory('sara_safety'),'config','mux.yaml')

    # Configuración del nodo multiplexor de topicos
    mux_node = Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            parameters=[mux_params, {'use_sim_time': use_sim_time}],
            remappings=[
            ('/cmd_vel_out', '/cmd_vel_in') ]
         )

    # Configuración del nodo de joystick
    # Maneja la entrada del control físico para teleoperación
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='teleop_joy',
        parameters=[{
            'dev': '/dev/input/js0',  # Ruta al dispositivo joystick
            'deadzone': 0.12    # Zona muerta para evitar ruido en las entradas
        }],
        respawn=True
    )

    
    # Nodo para limitar la velocidad del robot
    speed_limit_node= Node( package='sara_safety',executable='speed_limit.py')

    # Nodo para convertir señales del joystick en comandos de velocidad
    teleop_node= Node( package='sara_safety',executable='joy_teleop.py')

    # Nodo para evasion básica de obstáculos
    avoidance_node= Node( package='sara_safety',executable='naive_obstacle_avoidance.py')
    

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        mux_node,
        joy_node,
        speed_limit_node,
        teleop_node,
        avoidance_node,    
    ])
