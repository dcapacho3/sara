# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Archivo de lanzamiento para cartographer SLAM en robot SARA real
# Implementa la configuración de lanzamiento para un sistema de mapeo y localización
# simultánea (SLAM) utilizando Cartographer en el robot SARA en entorno real.
# La arquitectura configura los nodos necesarios para procesar datos de sensores físicos,
# generar mapas de ocupación y visualizar en tiempo real el proceso de mapeo.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    # Configuración del tiempo de ejecución
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    #Configuración de directorios y parámetros para Cartographer
    turtlebot3_cartographer_prefix = get_package_share_directory('sara_navigation')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  turtlebot3_cartographer_prefix, 'params'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='cartographer_params.lua')

    # Configuración de parámetros para el mapa de ocupación
    # Define la resolución y frecuencia de publicación del mapa generado   
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')


    # Configuración de RViz para visualización
    rviz_config_dir = os.path.join(get_package_share_directory('sara_navigation'),
                                   'rviz', 'cartographer_config.rviz')

    return LaunchDescription([
        # Declaración de argumentos para configuración de Cartographer
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Nodo principal de Cartographer
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),

        # Declaración de argumentos para el mapa de ocupación
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        # Inclusión del lanzador para el generador de mapa de ocupación
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec}.items(),
        ),

        # Nodo RViz para visualización
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])