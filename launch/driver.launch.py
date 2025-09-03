from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo para la primera cámara
        Node(
            package='hikvision_ros2_driver',
            executable='driver_node',
            name='camera1_node',
            parameters=[
                {'ip1': '192.168.196.105'},  # IP de la primera cámara
                {'ip2': '192.168.196.106'},      # IP de la segunda cámara
                {'username': 'admin'},        # Usuario (compartido)
                {'password': 'm6ai-lab'}      # Contraseña (compartida)
            ],
            output='screen'
        )
    ])