"""Launch user-specific calibration of kappa angle for RGB-D gaze"""

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
    user = LaunchConfiguration('user', default=os.path.join(get_package_share_directory(
        'ecard'), 'users', 'user0.yaml'))
    config_rviz2 = os.path.join(get_package_share_directory(
        'ecard'), 'config', 'rgbd_gaze', 'calibration', 'rviz2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'user',
            default_value=user,
            description='Path to config for RGB-D Gaze user'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ecard'), 'launch',
                              'rs', 'rs.launch.py')]),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ecard'), 'launch',
                              'rgbd_gaze', 'accuracy.launch.py')]),
            launch_arguments=[('user', user)],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ecard'), 'launch',
                              'rviz2.launch.py')]),
            launch_arguments=[('config_rviz2', config_rviz2)]
        ),
    ])
