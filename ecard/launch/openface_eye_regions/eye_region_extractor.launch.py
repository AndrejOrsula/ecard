"""Launch OpenFace Eye Region Extraction"""

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
            package='eye_region_extractor',
            node_executable='eye_region_extractor',
            node_name='eye_region_extractor',
            node_namespace='',
            output='screen',
            remappings=[('openface/landmarks_visible', 'ecard/landmarks_visible'),
                        ('openface/eye_regions', 'ecard/eye_regions')],
        ),
    ])
