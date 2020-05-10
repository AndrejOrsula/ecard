"""Launch calibration of eyeball for RGB-D gaze"""

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
    config_openface_separate = LaunchConfiguration('config_openface_separate', default=os.path.join(get_package_share_directory(
        'ecard'), 'config', 'openface', 'openface_separate_calibration.yaml'))
    config_openface_rgbd_head_pose = LaunchConfiguration('config_openface_rgbd_head_pose', default=os.path.join(get_package_share_directory(
        'ecard'), 'config', 'openface', 'openface_rgbd_head_pose.yaml'))
    config_rgbd_gaze_calibration_eyeball = LaunchConfiguration('config_rgbd_gaze_calibration_eyeball', default=os.path.join(get_package_share_directory(
        'ecard'), 'config', 'rgbd_gaze', 'calibration', 'rgbd_gaze_calibration_eyeball.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_openface_separate',
            default_value=config_openface_separate,
            description='Path to config for openface'),
        DeclareLaunchArgument(
            'config_openface_rgbd_head_pose',
            default_value=config_openface_rgbd_head_pose,
            description='Path to config for RGB-D head pose'),
        DeclareLaunchArgument(
            'config_rgbd_gaze_calibration_eyeball',
            default_value=config_rgbd_gaze_calibration_eyeball,
            description='Path to config for RGB-D Gaze calibration'),

        Node(
            package='openface',
            node_executable='openface_separate',
            node_name='openface_separate',
            node_namespace='',
            output='screen',
            parameters=[config_openface_separate],
            remappings=[('camera/image_raw', 'face_d415/camera/color/image_raw'),
                        ('camera/camera_info', 'face_d415/camera/color/camera_info'),
                        ('openface/landmarks_visible',
                         'ecard/rgbd_gaze/landmarks_visible'),
                        ('openface/head_pose', 'ecard/rgbd_gaze/head_pose_raw'),
                        ('openface/eye_landmarks_visible', 'ecard/rgbd_gaze/eye_landmarks_visible')],
        ),

        Node(
            package='openface_rgbd_head_pose',
            node_executable='openface_rgbd_head_pose',
            node_name='openface_rgbd_head_pose',
            node_namespace='',
            output='screen',
            parameters=[config_openface_rgbd_head_pose],
            remappings=[('camera/aligned_depth_to_color/image_raw',
                         'face_d415/camera/aligned_depth_to_color/image_raw'),
                        ('camera/aligned_depth_to_color/camera_info',
                         'face_d415/camera/aligned_depth_to_color/camera_info'),
                        ('openface/landmarks_visible',
                         'ecard/rgbd_gaze/landmarks_visible'),
                        ('openface/head_pose', 'ecard/rgbd_gaze/head_pose_raw'),
                        ('openface/rgbd_head_pose', 'ecard/rgbd_gaze/head_pose')],
        ),

        Node(
            package='openface_rgbd_eyelid_contour',
            node_executable='openface_rgbd_eyelid_contour',
            node_name='openface_rgbd_eyelid_contour',
            node_namespace='',
            output='screen',
            parameters=[],
            remappings=[('camera/aligned_depth_to_color/image_raw',
                         'face_d415/camera/aligned_depth_to_color/image_raw'),
                        ('camera/aligned_depth_to_color/camera_info',
                         'face_d415/camera/aligned_depth_to_color/camera_info'),
                        ('openface/eye_landmarks_visible',
                         'ecard/rgbd_gaze/eye_landmarks_visible'),
                        ('openface/eyelid_contours', 'ecard/rgbd_gaze/eyelid_contours')],
        ),

        Node(
            package='rgbd_gaze',
            node_executable='calibration_eyeball',
            node_name='rgbd_gaze_calibration_eyeball',
            node_namespace='',
            output='screen',
            parameters=[config_rgbd_gaze_calibration_eyeball],
            remappings=[('head_pose', 'ecard/rgbd_gaze/head_pose'),
                        ('eyelid_contours', 'ecard/rgbd_gaze/eyelid_contours'),
                        ('visualisation_markers',
                         'ecard/rgbd_gaze/visualisation_markers'),
                        ('eyelid_contours_cumulative', 'ecard/rgbd_gaze/eyelid_contours_cumulative')],
        ),
    ])
