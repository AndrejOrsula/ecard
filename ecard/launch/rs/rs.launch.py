"""Launch both RealSense cameras, i.e. D415 face and D435 scene, and configure them"""

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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/rs_face_d415.launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/rs_scene_d435.launch.py']),
        ),

        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='static_transform_publisher_face_d415_to_scene_d435',
            node_namespace='',
            output='screen',
            parameters=[],
            # Parallel
            arguments=['-0.05938306', '-0.0375', '0.0',
                       '3.14159265', '0.0', '0.0',
                       'face_d415', 'scene_d435'],
            # # # 15 deg offset
            # arguments=['-0.0212582', '-0.06153793', '0.0',
            #            '-3.40339204', '0.0', '0.0',
            #            'face_d415', 'scene_d435'],
            remappings=[],
        ),
    ])
