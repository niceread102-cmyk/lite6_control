from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_lite6_control',
            executable='sequence_sim',
            output='screen',
        ),
    ])
