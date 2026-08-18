from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="depth_image_proc",
            executable="point_cloud_xyzrgb_node",
            remappings=[
                ("rgb/image_rect_color", "/rgb"),
                ("rgb/camera_info", "/camera_info"),
                ("depth_registered/image_rect", "/depth"),
                ("points", "/phoxi/points_rgb"),
            ],
            parameters=[{
                "exact_sync": False,
                "queue_size": 30,
            }],
        ),
    ])
