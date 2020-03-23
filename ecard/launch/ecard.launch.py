"""Launch OpenFace with eye region extraction"""

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
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [os.path.join(get_package_share_directory('ecard'), 'launch',
        #                       'rs', 'rs.launch.py')]),
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ecard'), 'launch',
                              'openface_eye_regions', 'openface_separate.launch.py')]),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ecard'), 'launch',
                              'openface_eye_regions', 'eye_region_extractor.launch.py')]),
        ),

        Node(
            package='pupil_centre',
            node_executable='pupil_centre',
            node_name='pupil_centre',
            node_namespace='',
            output='screen',
            remappings=[('eye_regions', 'ecard/eye_regions')],
        ),
    ])
