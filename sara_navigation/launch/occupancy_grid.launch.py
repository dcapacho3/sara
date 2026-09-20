# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Archivo de lanzamiento para el generador de mapa de ocupación
# Implementa la configuración de lanzamiento para el nodo que convierte
# los datos de cartographer en un mapa de ocupación (occupancy grid) para
# el robot SARA. Este mapa es utilizado para visualización y planificación
# de rutas, representando el entorno como una rejilla 2D donde cada celda
# indica la probabilidad de estar ocupada por un obstáculo.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Configuración del tiempo de ejecucion
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # Configuración de la resolución del mapa
    resolution = LaunchConfiguration('resolution', default='0.05')
    # Configuración del período de publicación
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    return LaunchDescription([
        # Declaración del argumento de resolución
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        # Declaración del argumento de período de publicación
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),
        
        # Declaración del argumento de tiempo de simulación
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        # Nodo generador del mapa de ocupación
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]),
    ])