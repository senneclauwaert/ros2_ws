from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='straight_stop_pkg',
            executable='straight_stop',
            output='screen'),
    ])
