"""Launch OpenFace with eyelid contour extraction"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='openface_rgbd_eyelid_contour',
            node_executable='openface_rgbd_eyelid_contour',
            node_name='openface_rgbd_eyelid_contour',
            node_namespace='',
            output='screen',
            parameters=[],
            remappings=[('openface/eye_landmarks_visible', 'ecard/eye_landmarks_visible'),
                        ('camera/aligned_depth_to_color/image_raw',
                         'camera/aligned_depth_to_color/image_raw'),
                        ('camera/aligned_depth_to_color/camera_info',
                         'camera/aligned_depth_to_color/camera_info'),
                        ('openface/eyelid_contours', 'ecard/eyelid_contours')],
        ),
    ])
