from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_mediapipe_senne_pkg',
            executable='camera_detection',
            output='screen'),
        Node(
            package='lidar_mediapipe_senne_pkg',
            executable='obstacle_avoidance',
            output='screen'),
    ])