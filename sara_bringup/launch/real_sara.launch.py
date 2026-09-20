# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Archivo de lanzamiento para prueba de movimiento básico del robot SARA real
# Implementa la configuración de lanzamiento para visualizar y probar el movimiento
# básico del robot SARA en entorno real, utilizando el framework de transformaciones (TF)
# para representar las relaciones espaciales entre los diferentes componentes del robot.


import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import launch_ros
import os

def generate_launch_description():
    # Configuración de rutas a archivos y carpetas para el robot SARA
    # Define las ubicaciones de los modelos y configuraciones necesarias
    pkg_share = launch_ros.substitutions.FindPackageShare(package='sara_bringup').find('sara_bringup')
    description_share = launch_ros.substitutions.FindPackageShare(package='sara_description').find('sara_description')
    default_model_path = os.path.join(description_share, 'models/sara.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    
    
    # Nodo publicador del estado de las articulaciones
    # Genera estados para las articulaciones cuando no hay datos de sensores disponibles
    # Útil para pruebas básicas de movimiento sin hardware real
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', default_model_path]), value_type=str)}],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    
    # Nodo RViz para visualización
    # Proporciona una interfaz gráfica para observar el modelo 3D del robot SARA
    # y la representación visual de las transformaciones
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    # Creación y configuración de la descripción de lanzamiento
    # Configura los argumentos y nodos para la prueba de movimiento básico
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'),
        joint_state_publisher_node,

        rviz_node
    ])