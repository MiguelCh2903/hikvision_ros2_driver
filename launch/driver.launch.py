from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="hikvision_ros2_driver",
                executable="driver_node",
                name="hikvision_driver",
                parameters=[
                    # === Configuración de red ===
                    {"ip1": "192.168.196.104"},
                    {"ip2": "192.168.196.105"},
                    {"username": "admin"},
                    {"password": "m6ai-lab"},
                    # === Optimización para rosbag ===
                    {"jpeg_quality": 85},  # Calidad JPEG 1-100 (85 = buen balance)
                    {"use_compressed": True},  # True=compressed, False=raw
                ],
                output="screen",
            )
        ]
    )
