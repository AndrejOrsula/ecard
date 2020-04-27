"""Launch user-specific calibration for RGB-D gaze"""

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
    config_rviz2 = os.path.join(get_package_share_directory(
        'ecard'), 'config', 'rgbd_gaze', 'calibration', 'rviz2.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ecard'), 'launch',
                              'rs', 'rs_face_d415.launch.py')]),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ecard'), 'launch',
                              'rgbd_gaze', 'calibration.launch.py')]),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ecard'), 'launch',
                              'rviz2.launch.py')]),
            launch_arguments=[('config_rviz2', config_rviz2)]
        ),
    ])
