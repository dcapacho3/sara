# Autor: David Capacho Parra
# Descripción: Lanzamiento básico de SARA en simulación (Jazzy + Gazebo nuevo,
# Harmonic). Levanta el mundo en gz sim, publica robot_description con
# robot_state_publisher, genera (spawn) el robot desde ese mismo topic en
# lugar de un modelo SDF aparte, y hace de puente entre ROS y Gazebo
# Transport para cmd_vel/odom/scan/imu/tf.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

  pkg_share = get_package_share_directory('sara_bringup')
  description_dir = get_package_share_directory('sara_description')
  gazebo_dir = get_package_share_directory('sara_gazebo')
  default_model_path = os.path.join(description_dir, 'models', 'sara.urdf.xacro')
  default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')
  world_file_name = 'Supermarket.world'
  world_path = os.path.join(gazebo_dir, 'worlds', world_file_name)
  models_path = os.path.join(gazebo_dir, 'models')
  worlds_path = os.path.join(gazebo_dir, 'worlds')

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

  # The URDF->SDF conversion turns package://sara_description/... mesh URIs
  # into model://sara_description/..., which gz-sim resolves by looking for
  # a directory literally named "sara_description" somewhere on
  # GZ_SIM_RESOURCE_PATH. description_dir itself IS that directory
  # (.../share/sara_description), so its PARENT needs to be on the path,
  # not description_dir's own subfolders.
  gz_resource_path = (
    os.path.dirname(description_dir) + ':' +
    models_path + ':' + worlds_path + ':' +
    os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

  # gz sim replaces gzserver/gzclient. "-s" alone runs server-only
  # (headless); otherwise server and GUI come up together in one process.
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

  # Spawn from the /robot_description topic robot_state_publisher just
  # published, not a separate SDF file, so RViz/TF and the Gazebo spawn
  # always agree. Delayed so gz sim's spawn service is up first.
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

  # ROS <-> Gazebo Transport bridge for the topics the diff drive and
  # sensor plugins publish on the Gazebo side.
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
      # gz-sim's JointStatePublisher system plugin (sara.gazebo.xacro) publishes
      # this on the Gazebo side; without bridging it, robot_state_publisher never
      # gets wheel_left_joint/wheel_right_joint positions and can't broadcast
      # their TF, which is what made RViz report those links as disconnected.
      '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
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

  ld = LaunchDescription()

  ld.add_action(declare_model_path_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)
  ld.add_action(declare_use_rviz_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)

  ld.add_action(start_gz_sim_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(spawn_robot_cmd)
  ld.add_action(bridge_cmd)
  ld.add_action(start_rviz_cmd)

  return ld
