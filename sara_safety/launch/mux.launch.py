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

    zone_radius_m = LaunchConfiguration('zone_radius_m')
    max_obstacle_distance = LaunchConfiguration('max_obstacle_distance')
    min_obstacle_distance = LaunchConfiguration('min_obstacle_distance')
    min_standoff_m = LaunchConfiguration('min_standoff_m')
    reaction_time_s = LaunchConfiguration('reaction_time_s')
    human_approach_mps = LaunchConfiguration('human_approach_mps')
    max_linear_mps = LaunchConfiguration('max_linear_mps')
    payload_mass_kg = LaunchConfiguration('payload_mass_kg')

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
    avoidance_node = Node(
        package='sara_safety',
        executable='naive_obstacle_avoidance.py',
        parameters=[{'max_obstacle_distance': max_obstacle_distance,
                     'min_obstacle_distance': min_obstacle_distance}])

    # Nodo que lee la báscula del carrito (cart_weight_wrench) y controla la
    # velocidad máxima / bloqueo según el peso de carga detectado
    weight_monitor_node = Node(
        package='sara_safety',
        executable='weight_monitor.py',
        parameters=[{'use_sim_time': use_sim_time,
                     'startup_payload_kg': payload_mass_kg}])

    # Proximity Stop: detección por lidar (/scan), igual
    # que avoidance_node - deployable tal cual en sim y en el robot real, no
    # solo en un mundo de ensayo con un actor. Ver proximity_stop.py para
    # la derivación de zone_radius_m/min_valid_range_m contra el baseline
    # actual (robot_radius 0.28, lidar min range 0.4) y del cono frontal de
    # detección (reemplaza la exclusión de 90 grados que no cubría todo un
    # objeto estático cercano).
    proximity_stop_node = Node(
        package='sara_safety',
        executable='proximity_stop.py',
        parameters=[{'use_sim_time': use_sim_time,
                     'zone_radius_m': zone_radius_m}])

    # Adaptive Separation: límite de velocidad continuo
    # por lidar, se compone dentro de speed_limit.py vía min() (ver ese
    # archivo), NO es otro tópico de twist_mux - un límite continuo
    # dependiente de distancia no es un veto binario. d_min=zone_radius_m
    # actual de Proximity Stop (0.3m) para que las dos zonas empalmen sin
    # hueco - ver adaptive_separation.py para la derivación completa.
    adaptive_separation_node = Node(
        package='sara_safety',
        executable='adaptive_separation.py',
        parameters=[{'use_sim_time': use_sim_time,
                     'min_standoff_m': min_standoff_m,
                     'reaction_time_s': reaction_time_s,
                     'human_approach_mps': human_approach_mps,
                     'max_linear_mps': max_linear_mps}])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'zone_radius_m', default_value='0.3653',
            description="Proximity Stop's zone radius (m) - see this "
                         'file\'s comment above for why this exists as a '
                         'launch arg (ramp/tip-over trial overrides)'),
        DeclareLaunchArgument(
            'max_obstacle_distance', default_value='0.4',
            description='naive_obstacle_avoidance.py obstacle distance '
                         'ceiling (m), same reason as zone_radius_m'),
        DeclareLaunchArgument(
            'min_obstacle_distance', default_value='0.25',
            description='naive_obstacle_avoidance.py obstacle distance '
                         'floor (m), same reason as zone_radius_m'),
        DeclareLaunchArgument(
            'min_standoff_m', default_value='0.3653',
            description="Adaptive Separation's min standoff (m), same "
                         'reason as zone_radius_m'),
        DeclareLaunchArgument(
            'reaction_time_s', default_value='0.1',
            description="Adaptive Separation's reaction-time term (s), "
                         'same reason as zone_radius_m'),
        DeclareLaunchArgument(
            'human_approach_mps', default_value='1.4',
            description="Adaptive Separation's human-approach-speed term "
                         '(m/s), same reason as zone_radius_m'),
        DeclareLaunchArgument(
            'max_linear_mps', default_value='0.15',
            description="Adaptive Separation's own platform-top-speed "
                         'saturation ceiling $v_{plat}$ (m/s), same reason '
                         'as zone_radius_m - added 2026-09-30 specifically '
                         "to demonstrate this parameter's effect on the "
                         'width of the graded speed-vs-range transition '
                         '(paper2_draft.tex Section 6.2); SARA-real value '
                         'stays 0.15, unaffected unless explicitly '
                         'overridden'),
        DeclareLaunchArgument(
            'payload_mass_kg', default_value='0.0',
            description='Payload mass (kg) already present on the cart at '
                         'spawn time - forwarded to weight_monitor.py as '
                         'startup_payload_kg so its auto-tare does not '
                         "zero out a payload that's there from t=0"),
        mux_node,
        joy_node,
        speed_limit_node,
        teleop_node,
        # avoidance_node (naive_obstacle_avoidance.py)
        weight_monitor_node,
        proximity_stop_node,
        adaptive_separation_node,
    ])
