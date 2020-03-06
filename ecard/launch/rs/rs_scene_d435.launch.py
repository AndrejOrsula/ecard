"""Launch RealSense D435 perceiving scene"""

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

    config_rs_scene_d435 = LaunchConfiguration('config_rs_scene_d435', default=os.path.join(get_package_share_directory(
        'ecard'), 'config', 'rs', 'rs_scene_d435.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_rs_scene_d435',
            default_value=config_rs_scene_d435,
            description='Path to config for D435 scene camera'),

        Node(
            package='realsense_node',
            node_executable='realsense_node',
            node_name='realsense_node',
            node_namespace="",
            output='screen',
            parameters=[config_rs_scene_d435],
            arguments=[],
            remappings=[],
        ),
    ])
