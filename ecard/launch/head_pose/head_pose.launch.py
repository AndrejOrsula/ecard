"""Launch OpenVINO pipeline"""

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
    config_openvino_pipeline = LaunchConfiguration('config_openvino_pipeline', default=os.path.join(get_package_share_directory('ecard'), 'config', 'head_pose',
                                                                                                    'pipeline.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_openvino_pipeline',
            default_value=config_openvino_pipeline,
            description='Path to config for OpenVINO pipeline'),

        Node(
            package='dynamic_vino_sample',
            node_executable='pipeline_with_params',
            node_name='head_pose_estimator',
            node_namespace="",
            output='screen',
            parameters=[],
            arguments=['-config', config_openvino_pipeline],
            remappings=[
                ('/openvino_toolkit/ecard/detected_objects', '/ecard/faces'),
                ('/openvino_toolkit/ecard/detected_landmarks', '/ecard/face_landmarks'),
                ('/openvino_toolkit/ecard/headposes', '/ecard/headposes'),
                ('/openvino_toolkit/ecard/images', '/ecard/images')],
        )
    ])
