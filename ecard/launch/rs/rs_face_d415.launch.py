"""Launch RealSense D415 perceiving user's face"""

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

    config_rs_face_d415 = LaunchConfiguration('config_rs_face_d415', default=os.path.join(get_package_share_directory(
        'ecard'), 'config', 'rs', 'rs_face_d415.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_rs_face_d415',
            default_value=config_rs_face_d415,
            description='Path to config for D415 face camera'),

        Node(
            package='realsense_node',
            node_executable='realsense_node',
            node_name='realsense_node',
            node_namespace='/face_d415',
            output='screen',
            parameters=[config_rs_face_d415],
            arguments=[],
            remappings=[],
        ),
    ])
