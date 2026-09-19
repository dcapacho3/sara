# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Archivo de lanzamiento para navegación Nav2 del robot SARA en entorno real
# Implementa la configuración de lanzamiento para un sistema de navegación autónoma
# utilizando Nav2 en el robot SARA real (hardware). La arquitectura integra localización
# mediante AMCL, carga de mapas previamente generados con enmascaramiento para áreas
# prohibidas, visualización, y filtros de costo para una navegación segura y eficiente.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers.on_process_start import OnProcessStart
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():

  # Configuración de rutas a archivos y carpetas
  # Define las ubicaciones de modelos, configuraciones y mapas para la navegación
  pkg_share = FindPackageShare(package='sara').find('sara')
  default_model_path = os.path.join(pkg_share, 'models/sara.urdf.xacro')  

  default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2_config_v2.rviz')
  nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
  nav2_launch_dir = os.path.join(nav2_dir, 'launch') 
  static_map_path = os.path.join(pkg_share, 'maps', 'labrobfinal_mask.yaml')
  nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params_super.yaml')
  nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
  behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
  
  # Configuración para el filtro de Kalman Extendido (EKF)
  ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

  # Variables de configuración del lanzamiento
  # Parámetros configurables para la navegación en entorno real
  autostart = LaunchConfiguration('autostart')
  default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
  headless = LaunchConfiguration('headless')
  map_yaml_file = LaunchConfiguration('map')
  model = LaunchConfiguration('model')
  namespace = LaunchConfiguration('namespace')
  params_file = LaunchConfiguration('params_file')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  slam = LaunchConfiguration('slam')
  use_namespace = LaunchConfiguration('use_namespace')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
  
  # Configuración de remapeos para transformaciones
  # Asegura la correcta comunicación entre los nodos del sistema
  remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]
  
  # Declaración de los argumentos de lanzamiento
  # Permite configurar el comportamiento del sistema en tiempo de ejecución
  declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace')

  declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',
    default_value='True',
    description='Whether to apply a namespace to the navigation stack')
        
  declare_autostart_cmd = DeclareLaunchArgument(
    name='autostart', 
    default_value='true',
    description='Automatically startup the nav2 stack')

  declare_bt_xml_cmd = DeclareLaunchArgument(
    name='default_bt_xml_filename',
    default_value=behavior_tree_xml_path,
    description='Full path to the behavior tree xml file to use')
        
  declare_map_yaml_cmd = DeclareLaunchArgument(
    name='map',
    default_value=static_map_path,
    description='Full path to map file to load')
        
  declare_model_path_cmd = DeclareLaunchArgument(
    name='model', 
    default_value=default_model_path, 
    description='Absolute path to robot urdf file')
    
  declare_params_file_cmd = DeclareLaunchArgument(
    name='params_file',
    default_value=nav2_params_path,
    description='Full path to the ROS2 parameters file to use for all launched nodes')
    
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='False',
    description='Use simulation (Gazebo) clock if true')
  
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')

  declare_slam_cmd = DeclareLaunchArgument(
    name='slam',
    default_value='False',
    description='Whether to run SLAM')
    
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
    
  # Publicador del estado del robot
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    namespace=namespace,
    parameters=[{'use_sim_time': use_sim_time,
    'robot_description': ParameterValue(Command(['xacro ', model]), value_type=str)}],
    remappings=remappings,
    arguments=[default_model_path])

  # Lanzamiento de RViz para visualización
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])    
    
  # Nodo AMCL para localización
  # Implementa la localización Monte Carlo adaptativa para posicionamiento
  start_amcl_cmd = Node(
  	package='nav2_amcl',
 	  executable='amcl',
  	name='amcl',
  	output='screen',
  	parameters=[nav2_params_path],
  	remappings=[('/scan', 'scan')]
	)


  # Nodo de localización mediante EKF
  # Fusiona datos de odometría y amcl para estimación precisa y fluida de pose
  start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[ekf_config_path]
  )

  # Nodo limpiador de mapas de costo
  # Mejora la calidad de los mapas de navegación eliminando ruido
  costmap_cleaner_node = Node(package='sara', executable='costmap_cleaner.py')

  # Nodo bloqueador de multiplexor por medio de balanza integrada
  locker_node = Node(package='sara', executable='mux_locker.py')
    
  # Lanzamiento de la pila de navegación ROS 2
  # Configura e inicia todos los componentes principales del sistema Nav2
  start_ros2_navigation_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
    launch_arguments = {'namespace': namespace,
                        'use_namespace': use_namespace,
                        'slam': slam,
                        'map': map_yaml_file,
                        'use_sim_time': use_sim_time,
                        'params_file': params_file,
                        'default_bt_xml_filename': default_bt_xml_filename,
                        'autostart': autostart}.items())

  # Creación de la descripción de lanzamiento y población
  ld = LaunchDescription()

  # Declaración de las opciones de lanzamiento
  # Añade todos los argumentos definidos anteriormente
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_autostart_cmd)
  ld.add_action(declare_bt_xml_cmd)
  ld.add_action(declare_map_yaml_cmd)
  ld.add_action(declare_model_path_cmd)
  ld.add_action(declare_params_file_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_slam_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)

  # Adición de las acciones a ejecutar
  # Configura la secuencia de lanzamiento para el robot SARA real
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_rviz_cmd)
  ld.add_action(start_ros2_navigation_cmd)
  ld.add_action(locker_node)
  ld.add_action(start_amcl_cmd)
  ld.add_action(start_robot_localization_cmd)
  ld.add_action(costmap_cleaner_node)  
 
  return ld