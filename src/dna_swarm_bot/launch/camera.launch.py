import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('image_size', default_value='[640, 480]'),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'image_size': LaunchConfiguration('image_size'),
                'camera_frame_id': 'camera_link_optical',
            }],
            remappings=[('image_raw', 'camera/image_raw'),
                        ('camera_info', 'camera/camera_info')]
        ),
    ])
