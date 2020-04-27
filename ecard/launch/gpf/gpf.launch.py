"""Launch Geometric Primitive Fitting"""

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
    config_gpf = LaunchConfiguration('config_gpf', default=os.path.join(get_package_share_directory(
        'ecard'), 'config', 'gpf', 'gpf.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_gpf',
            default_value=config_gpf,
            description='Path to config for GPF'),

        Node(
            package='gpf',
            node_executable='gpf',
            node_name='gpf',
            node_namespace='',
            output='screen',
            parameters=[config_gpf],
            remappings=[('camera/pointcloud', 'scene_d435/camera/pointcloud'),
                        ('visualisation_markers', 'ecard/gpf/visualisation_markers'),
                        ('geometric_primitives', 'ecard/gpf/geometric_primitives')],
        ),
    ])
