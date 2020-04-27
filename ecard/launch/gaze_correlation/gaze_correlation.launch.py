"""Launch Gaze Correlation"""

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
    config_gaze_correlation = LaunchConfiguration('config_gaze_correlation', default=os.path.join(get_package_share_directory(
        'ecard'), 'config', 'gaze_correlation', 'gaze_correlation.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_gaze_correlation',
            default_value=config_gaze_correlation,
            description='Path to config for gaze_correlation'),

        Node(
            package='gaze_correlation',
            node_executable='gaze_correlation',
            node_name='gaze_correlation',
            node_namespace='',
            output='screen',
            parameters=[config_gaze_correlation],
            remappings=[('gaze', 'ecard/rgbd_gaze/compound_gaze'),
                        ('geometric_primitives', 'ecard/gpf/geometric_primitives'),
                        ('object_of_interest', 'ecard/gaze_correlation/object_of_interest'),
                        ('visualisation_markers', 'ecard/gaze_correlation/visualisation_markers')],
        ),
    ])
