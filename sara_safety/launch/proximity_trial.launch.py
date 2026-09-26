# Autor: David Capacho Parra
# Descripción: Launch de ensayo para Proximity Stop (paper 2, sec 3.1 /
# paper_outline.md Phase 2 y 4). Self-contained: Gazebo + ProximityTrial.world
# (robot + un actor humano nativo de gz-sim, ver ese archivo) + el stack de
# seguridad completo (mux.launch.py, que ya incluye proximity_stop.py - es
# un nodo deployable por lidar, igual que avoidance_node, no algo exclusivo
# de este mundo de ensayo).
#
# No hay bridge de ground-truth del actor humano aquí - se intentó
# (plugin en el actor, luego bridgear pose/info y dynamic_pose/info) y los
# tres caminos están confirmados bloqueados por una limitación real de
# gz-sim8 (actores no expuestos en ningún topic de pose estándar del scene
# broadcaster), ver el comentario junto a bridge_cmd abajo para el detalle
# completo. Section 5/6 del paper usan la distancia detectada por el lidar
# directamente, sin cruce contra ground-truth del actor.
#
# El actor humano no necesita un nodo aparte manejando su trayectoria en
# tiempo de ejecución (human_actor_driver.py, eliminado): su trayectoria
# vive directamente en ProximityTrial.world como un <actor><script>
# nativo de gz-sim, que la interpola internamente sin ningún polling
# externo - ver el encabezado de ese archivo para el porqué.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_dir = get_package_share_directory('sara_description')
    gazebo_dir = get_package_share_directory('sara_gazebo')
    safety_dir = get_package_share_directory('sara_safety')
    default_model_path = os.path.join(description_dir, 'models', 'sara.urdf.xacro')
    world_path = os.path.join(gazebo_dir, 'worlds', 'ProximityTrial.world')
    models_path = os.path.join(gazebo_dir, 'models')
    worlds_path = os.path.join(gazebo_dir, 'worlds')

    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    payload_mass_kg = LaunchConfiguration('payload_mass_kg')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_pitch = LaunchConfiguration('spawn_pitch')
    zone_radius_m = LaunchConfiguration('zone_radius_m')
    max_obstacle_distance = LaunchConfiguration('max_obstacle_distance')
    min_obstacle_distance = LaunchConfiguration('min_obstacle_distance')
    min_standoff_m = LaunchConfiguration('min_standoff_m')
    reaction_time_s = LaunchConfiguration('reaction_time_s')
    human_approach_mps = LaunchConfiguration('human_approach_mps')

    declare_model_path_cmd = DeclareLaunchArgument(
        name='model', default_value=default_model_path,
        description='Absolute path to robot urdf/xacro file')

    declare_world_cmd = DeclareLaunchArgument(
        name='world', default_value=world_path,
        description='Full path to the world model file to load')

    declare_payload_mass_cmd = DeclareLaunchArgument(
        name='payload_mass_kg', default_value='0.0',
        description='Extra payload mass (kg) added to base_weight at spawn '
                     'time - see sara.urdf.xacro for why this exists (real '
                     'mass for loaded-braking trials, not a wrench)')

    declare_spawn_x_cmd = DeclareLaunchArgument(
        name='spawn_x', default_value='0.0', description='Spawn x position (m)')
    declare_spawn_z_cmd = DeclareLaunchArgument(
        name='spawn_z', default_value='0.1', description='Spawn z position (m)')
    declare_spawn_pitch_cmd = DeclareLaunchArgument(
        name='spawn_pitch', default_value='0.0',
        description='Spawn pitch (rad) - for slope/tip-over trials, and for '
                     'verifying /world/default/pose/info orientation '
                     'conventions against a known value')
    declare_zone_radius_cmd = DeclareLaunchArgument(
        name='zone_radius_m', default_value='1.2',
        description="Proximity Stop's zone radius override - see "
                     "mux.launch.py for why (slope/tip-over worlds have a "
                     "large static ramp deliberately close to the robot, "
                     "which Proximity Stop would otherwise correctly veto)")
    declare_max_obstacle_cmd = DeclareLaunchArgument(
        name='max_obstacle_distance', default_value='0.4',
        description='naive_obstacle_avoidance.py override - same reason as '
                     'zone_radius_m above (a second, separate lidar-based '
                     'node with its own threshold, priority 200 - also '
                     'blocks slope/tip-over trials otherwise)')
    declare_min_obstacle_cmd = DeclareLaunchArgument(
        name='min_obstacle_distance', default_value='0.25',
        description='naive_obstacle_avoidance.py override, same reason as '
                     'max_obstacle_distance above')
    declare_min_standoff_cmd = DeclareLaunchArgument(
        name='min_standoff_m', default_value='1.2',
        description="Adaptive Separation's min standoff override - a THIRD "
                     "independent lidar-based layer, same reason as "
                     "zone_radius_m above")
    declare_reaction_time_cmd = DeclareLaunchArgument(
        name='reaction_time_s', default_value='0.3',
        description="Adaptive Separation's reaction-time term override - "
                     "min_standoff_m alone is not sufficient, see "
                     "mux.launch.py's adaptive_separation_node comment")
    declare_human_approach_cmd = DeclareLaunchArgument(
        name='human_approach_mps', default_value='1.4',
        description="Adaptive Separation's human-approach-speed term "
                     "override - min_standoff_m alone is not sufficient, "
                     "see mux.launch.py's adaptive_separation_node comment")

    gz_resource_path = (
        os.path.dirname(description_dir) + ':' +
        models_path + ':' + worlds_path + ':' +
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

    start_gz_sim_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '3', world],
        additional_env={'GZ_SIM_RESOURCE_PATH': gz_resource_path},
        output='screen')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True,
            'robot_description': ParameterValue(
                Command(['xacro ', model, ' payload_mass_kg:=', payload_mass_kg]),
                value_type=str)}])

    spawn_robot_cmd = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-name', 'sara', '-topic', 'robot_description',
                           '-x', spawn_x, '-y', '0', '-z', spawn_z,
                           '-P', spawn_pitch],
                output='screen')
        ])

    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel_out@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # cart_weight_wrench: the force_torque sensor on cart_weight_joint
            # (sara.gazebo.xacro) that weight_monitor.py reads for Payload
            # Interaction. Missing from this list until now - a real gap,
            # not by design: weight_monitor.py was silently stuck in its
            # warmup/tare wait forever under this launch file (confirmed
            # empirically: 71s of live sim time, zero cart_cargo_weight_kg
            # messages, with the raw gz-side /cart_weight_wrench topic
            # publishing correctly the whole time).
            '/cart_weight_wrench@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
            # ROS->Gazebo only ("]"): lets a ROS node drive controlled payload
            # weight/rate stimuli for Payload Interaction / Adaptive
            # Separation trials, through the world's gz-sim-apply-link-wrench-
            # system plugin. A persistent rclpy publisher, not repeated `gz
            # topic pub` CLI calls: each CLI invocation spawns a fresh
            # gz-transport process with its own peer-discovery handshake
            # (confirmed empirically - ~0.9s overhead per call, turning an
            # intended 8s ramp into 26.76s wall-clock and making delivery
            # timing too unpredictable for any real rate control), whereas a
            # long-lived publisher pays that discovery cost once.
            '/world/default/wrench/persistent@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
            # Persistent wrenches on the same entity are ADDITIVE (confirmed
            # empirically - each publish adds a new contribution, does not
            # replace the previous one), so a removal-event stimulus needs
            # this to reset to zero rather than publishing a cancelling
            # negative force (which would itself register as a second,
            # separate weight-change event).
            '/world/default/wrench/clear@ros_gz_interfaces/msg/Entity]gz.msgs.Entity',
        ],
        # No ground-truth bridge for the human actor: confirmed empirically,
        # live, that gz-sim8's scene broadcaster does not expose <actor>
        # entities on ANY standard per-entity pose topic (tried the
        # gz-sim-odometry-publisher-system plugin on the actor directly -
        # fails, requires a <model> entity; tried bridging both pose/info
        # and dynamic_pose/info - neither ever lists "human_actor" among
        # their entities, confirmed via `gz topic -e | grep name:` while a
        # trial was live). This is a documented gz-sim8 limitation, not a
        # bridge misconfiguration - see run_proximity_trial.py and
        # paper2_draft.tex Section 6 for how it's honestly reported. The
        # robot's own ground truth (a <model>) is unaffected.
        parameters=[{'use_sim_time': True}],
        output='screen')

    mux_stack_cmd = TimerAction(
        period=1.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(safety_dir, 'launch', 'mux.launch.py')),
            launch_arguments={'use_sim_time': 'true',
                               'zone_radius_m': zone_radius_m,
                               'max_obstacle_distance': max_obstacle_distance,
                               'min_obstacle_distance': min_obstacle_distance,
                               'min_standoff_m': min_standoff_m,
                               'reaction_time_s': reaction_time_s,
                               'human_approach_mps': human_approach_mps}.items())])

    ld = LaunchDescription()
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_payload_mass_cmd)
    ld.add_action(declare_spawn_x_cmd)
    ld.add_action(declare_spawn_z_cmd)
    ld.add_action(declare_spawn_pitch_cmd)
    ld.add_action(declare_zone_radius_cmd)
    ld.add_action(declare_max_obstacle_cmd)
    ld.add_action(declare_min_obstacle_cmd)
    ld.add_action(declare_min_standoff_cmd)
    ld.add_action(declare_reaction_time_cmd)
    ld.add_action(declare_human_approach_cmd)
    ld.add_action(start_gz_sim_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(bridge_cmd)
    ld.add_action(mux_stack_cmd)
    return ld
