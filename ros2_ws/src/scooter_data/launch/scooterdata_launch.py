from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scooter_data',
            executable='camera',
            name='camera',
        ),
        Node(
            package='scooter_data',
            executable='radar',
            name='radar',
        ),
        Node(
            package='scooter_data',
            executable='servo',
            name='servo',
        ),
        Node(
            package='scooter_data',
            executable='camera_yolov26_node',
            name='camera_yolov26_node',
        )
    ])