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
    config_pupil_centre = LaunchConfiguration('config_pupil_centre', default=os.path.join(get_package_share_directory(
        'pupil_centre'), 'config', 'pupil_centre.yaml'))
    config_rgbd_gaze = LaunchConfiguration('config_rgbd_gaze', default=os.path.join(get_package_share_directory(
        'rgbd_gaze'), 'config', 'rgbd_gaze.yaml'))
    config_rgbd_gaze_user = LaunchConfiguration('config_rgbd_gaze_user', default=os.path.join(get_package_share_directory(
        'rgbd_gaze'), 'config', 'rgbd_gaze_user.yaml'))
    config_rgbd_gaze_calibration = LaunchConfiguration('config_rgbd_gaze_calibration', default=os.path.join(get_package_share_directory(
        'rgbd_gaze'), 'config', 'rgbd_gaze_calibration.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_pupil_centre',
            default_value=config_pupil_centre,
            description='Path to config for pupil centre localisation'),
        DeclareLaunchArgument(
            'config_rgbd_gaze',
            default_value=config_rgbd_gaze,
            description='Path to config for RGB-D Gaze'),
        DeclareLaunchArgument(
            'config_rgbd_gaze_user',
            default_value=config_rgbd_gaze_user,
            description='Path to config for RGB-D Gaze user'),
        DeclareLaunchArgument(
            'config_rgbd_gaze_calibration',
            default_value=config_rgbd_gaze_calibration,
            description='Path to config for RGB-D Gaze calibration'),



        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [os.path.join(get_package_share_directory('ecard'), 'launch',
        #                       'rs', 'rs.launch.py')]),
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ecard'), 'launch',
                              'openface', 'openface_separate.launch.py')]),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ecard'), 'launch',
                              'openface', 'openface_eye_region.launch.py')]),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ecard'), 'launch',
                              'openface', 'openface_rgbd_head_pose.launch.py')]),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ecard'), 'launch',
                              'openface', 'openface_rgbd_eyelid_contour.launch.py')]),
        ),


        Node(
            package='pupil_centre',
            node_executable='pupil_centre',
            node_name='pupil_centre',
            node_namespace='',
            output='screen',
            parameters=[config_pupil_centre],
            remappings=[('eye_regions', 'ecard/eye_regions'),
                        ('camera/aligned_depth_to_color/image_raw',
                         'camera/aligned_depth_to_color/image_raw'),
                        ('camera/aligned_depth_to_color/camera_info',
                         'camera/aligned_depth_to_color/camera_info'),
                        ('pupil_centres', 'ecard/pupil_centres')],
        ),

        Node(
            package='rgbd_gaze',
            node_executable='calibration',
            node_name='rgbd_gaze_calibration',
            node_namespace='',
            output='screen',
            parameters=[config_rgbd_gaze_calibration],
            remappings=[('head_pose', 'ecard/head_pose'),
                        ('pupil_centres', 'ecard/pupil_centres'),
                        ('eyelid_contours', 'ecard/eyelid_contours')],
        ),

        # Node(
        #     package='rgbd_gaze',
        #     node_executable='rgbd_gaze',
        #     node_name='rgbd_gaze',
        #     node_namespace='',
        #     output='screen',
        #     parameters=[config_rgbd_gaze, config_rgbd_gaze_user],
        #     remappings=[('head_pose', 'ecard/head_pose'),
        #                 ('pupil_centres', 'ecard/pupil_centres')],
        # ),
    ])
