# Sim navigation entry point for SARA (Jazzy + new Gazebo). This is the
# launch file base_navgui.py actually starts for simulated runs: world +
# robot spawn + sensor bridge, Nav2 bringup on the existing supermarket map,
# EKF sensor fusion, and an initial pose seed for AMCL.
#
# The old mecanum ros2_control path (agv_control_navigation.launch.py /
# robot_control_navigation.py, forward_velocity_controller,
# joint_state_broadcaster) is intentionally dropped: there was never a
# controller_manager or <ros2_control> tag anywhere in this repo backing it,
# confirmed by a repo-wide grep, so it was already dead code before this
# migration, not something the migration broke.

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
    bringup_dir = get_package_share_directory('turtlemart')
    launch_dir = os.path.join(bringup_dir, 'launch')
    default_model_path = os.path.join(bringup_dir, 'models', 'turtlemart.urdf.xacro')
    world_file_name = 'turtlemart_world/Supermarket.world'
    world_path = os.path.join(bringup_dir, 'worlds', world_file_name)
    models_path = os.path.join(bringup_dir, 'models')
    worlds_path = os.path.join(bringup_dir, 'worlds')
    ekf_config_path = os.path.join(bringup_dir, 'config', 'ekf.yaml')

    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    model = LaunchConfiguration('model')

    default_rviz_config_path = os.path.join(bringup_dir, 'rviz', 'nav2_config_v2.rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')

    declare_model_path_cmd = DeclareLaunchArgument(
        name='model', default_value=default_model_path,
        description='Absolute path to robot urdf/xacro file')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'supermarket_map.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params_sim.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file', default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator', default_value='True',
        description='Whether to start the simulator')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub', default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to start RVIZ')

    declare_world_cmd = DeclareLaunchArgument(
        name='world', default_value=world_path,
        description='Full path to the world model file to load')

    gz_resource_path = (
        os.path.dirname(bringup_dir) + ':' +
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
                    '-name', 'turtlemart',
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

    # Delayed a few seconds behind gz sim's own start. Nav2's lifecycle
    # managers use a wall-clock bond_timeout (4.0s default, not overridable
    # here since nav2_bringup's stock localization/navigation launch files
    # don't read it from params_file) to detect dead nodes via heartbeat.
    # Starting the whole Nav2 stack at the exact same instant as Gazebo's
    # own cold start (world parse, physics init, rendering — the heaviest
    # CPU moment of the launch) risks missing that heartbeat under load,
    # which reads as Nav2 intermittently failing to come up depending on
    # how loaded the machine happens to be at that moment.
    bringup_cmd = TimerAction(
        period=4.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
            launch_arguments={'namespace': '',
                              'use_namespace': 'false',
                              'slam': slam,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'params_file': params_file,
                              'default_bt_xml_filename': default_bt_xml_filename,
                              'autostart': autostart}.items())])

    start_robot_localization_cmd = Node(
      package='robot_localization',
      executable='ekf_node',
      name='ekf_filter_node',
      output='screen',
      parameters=[ekf_config_path, {'use_sim_time': use_sim_time}])

    # Delayed so AMCL (started by bringup_cmd) is already up and subscribed
    # to /initialpose before this one-shot publish fires; otherwise the
    # message can be lost and AMCL never gets seeded, same as any ROS
    # publish-before-subscriber-connects race.
    initial_pose_cmd = TimerAction(
        period=10.0,
        actions=[Node(package='turtlemart', executable='initial_pose_pub.py',
                       parameters=[{'use_sim_time': use_sim_time}])])

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_world_cmd)

    ld.add_action(start_gz_sim_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(bridge_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(initial_pose_cmd)

    return ld
