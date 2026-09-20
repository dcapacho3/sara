# Autor: David Capacho Parra
# Descripción: SLAM con Cartographer sobre SARA en simulación (Jazzy + Gazebo
# nuevo, Harmonic). Mismo patrón world/spawn/bridge que sara.launch.py
# y navagv.launch.py: gz sim, robot_state_publisher publica robot_description,
# el robot se genera (spawn) desde ese mismo topic, y un puente ROS<->Gazebo
# Transport conecta cmd_vel/odom/scan/imu/tf/clock.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = get_package_share_directory('sara_navigation')
    description_dir = get_package_share_directory('sara_description')
    gazebo_dir = get_package_share_directory('sara_gazebo')
    default_model_path = os.path.join(description_dir, 'models', 'sara.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'cartographer_config.rviz')
    world_file_name = 'Supermarket.world'
    world_path = os.path.join(gazebo_dir, 'worlds', world_file_name)
    models_path = os.path.join(gazebo_dir, 'models')
    worlds_path = os.path.join(gazebo_dir, 'worlds')

    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  pkg_share, 'params'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='cartographer_params.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    headless = LaunchConfiguration('headless')
    model = LaunchConfiguration('model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')

    declare_model_path_cmd = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf/xacro file')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to run gz sim server-only, without the GUI')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')

    declare_cartographer_dir_cmd = DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load')

    declare_cartographer_param_cmd = DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer')

    declare_resolution = DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid')

    decalare_publish_period = DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period')

    gz_resource_path = (
        os.path.dirname(description_dir) + ':' +
        models_path + ':' + worlds_path + ':' +
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

    start_gz_sim_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gz', 'sim', '-r', '-v', '4', world],
        additional_env={'GZ_SIM_RESOURCE_PATH': gz_resource_path},
        output='screen')

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(Command(['xacro ', model]), value_type=str)}])

    spawn_robot_cmd = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'sara',
                    '-topic', 'robot_description',
                    '-x', '0.5', '-y', '-3.0', '-z', '0.1',
                    '-Y', '1.58',
                ],
                output='screen')
        ])

    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel_out@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])

    start_cartographer_cmd = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename])

    start_ocupancy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'occupancy_grid.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'resolution': resolution,
            'publish_period_sec': publish_period_sec
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_cartographer_dir_cmd)
    ld.add_action(declare_cartographer_param_cmd)
    ld.add_action(declare_resolution)
    ld.add_action(decalare_publish_period)

    ld.add_action(start_gz_sim_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(bridge_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_cartographer_cmd)
    ld.add_action(start_ocupancy)

    return ld
