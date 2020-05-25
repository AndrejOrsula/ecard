"""Launch calibration of kappa for RGB-D gaze"""

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
        'ecard'), 'users', 'default.yaml'))
    config_rgbd_gaze_calibration_kappa = LaunchConfiguration('config_rgbd_gaze_calibration_kappa', default=os.path.join(get_package_share_directory(
        'ecard'), 'config', 'rgbd_gaze', 'calibration', 'rgbd_gaze_calibration_kappa.yaml'))
    config_simple_marker_detector = LaunchConfiguration('config_simple_marker_detector', default=os.path.join(get_package_share_directory(
        'ecard'), 'config', 'rgbd_gaze', 'calibration', 'simple_marker_detector.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'user',
            default_value=user,
            description='Path to config for user of RGB-D gaze'),
        DeclareLaunchArgument(
            'config_rgbd_gaze_calibration_kappa',
            default_value=config_rgbd_gaze_calibration_kappa,
            description='Path to config for RGB-D Gaze calibration for eyeball'),
        DeclareLaunchArgument(
            'config_simple_marker_detector',
            default_value=config_simple_marker_detector,
            description='Path to config for simple marker detector'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ecard'), 'launch',
                              'rgbd_gaze', 'rgbd_gaze.launch.py')]),
            launch_arguments=[('user', user)],
        ),

        Node(
            package='rgbd_gaze',
            node_executable='calibration_kappa',
            node_name='rgbd_gaze_calibration_kappa',
            node_namespace='',
            output='screen',
            parameters=[config_rgbd_gaze_calibration_kappa],
            remappings=[('head_pose', 'ecard/rgbd_gaze/head_pose'),
                        ('optical_axes', 'ecard/rgbd_gaze/optical_axes'),
                        ('scene_point', 'ecard/rgbd_gaze/scene_point'),
                        ('visualisation_markers', 'ecard/rgbd_gaze/visualisation_markers')],
        ),

        Node(
            package='simple_marker_detector',
            node_executable='simple_marker_detector',
            node_name='simple_marker_detector',
            node_namespace='',
            output='screen',
            parameters=[config_simple_marker_detector],
            remappings=[('camera/color/image_raw', 'scene_d435/camera/color/image_raw'),
                        ('camera/aligned_depth_to_color/image_raw',
                         'scene_d435/camera/aligned_depth_to_color/image_raw'),
                        ('camera/aligned_depth_to_color/camera_info',
                         'scene_d435/camera/aligned_depth_to_color/camera_info'),
                        ('marker_centre', 'ecard/rgbd_gaze/scene_point')],
        ),
    ])
