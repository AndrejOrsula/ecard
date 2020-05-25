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
    user = LaunchConfiguration('user', default=os.path.join(get_package_share_directory(
        'ecard'), 'users', 'user0.yaml'))

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
                              'rgbd_gaze', 'rgbd_gaze.launch.py')]),
            launch_arguments=[('user', user)],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ecard'), 'launch',
                              'gpf', 'gpf.launch.py')]),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ecard'), 'launch',
                              'gaze_correlation', 'gaze_correlation.launch.py')]),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ecard'), 'launch',
                              'manipulation.launch.py')]),
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [os.path.join(get_package_share_directory('ecard'), 'launch',
        #                       'rviz2.launch.py')]),
        # ),

        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='static_transform_publisher_world_face_d415',
            node_namespace='',
            output='screen',
            parameters=[],
            arguments=['0.1', '0.4', '0.25',
                       '0.0', '3.14159265', '3.14159265',
                       'world', 'face_d415'],
            remappings=[],
        ),
    ])
