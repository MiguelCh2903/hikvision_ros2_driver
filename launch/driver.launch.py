from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Ruta al archivo de configuración YAML
    config_file = PathJoinSubstitution([
        FindPackageShare('hikvision_ros2_driver'),
        'config',
        'camera_params.yaml'
    ])
    
    return LaunchDescription(
        [
            Node(
                package="hikvision_ros2_driver",
                executable="driver_node",
                name="hikvision_driver",
                parameters=[config_file],
                output="screen",
            )
        ]
    )
