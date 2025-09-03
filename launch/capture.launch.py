from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hikvision_ros2_driver',
            executable='capture_node',
            name='capture_node',
            output='screen'
        )
    ])
