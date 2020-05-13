import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    moveit_cpp_yaml_file_name = get_package_share_directory(
        'ecard') + "/config/moveit/moveit_cpp.yaml"
    robot_description_config = load_file(
        'ecard', 'config/moveit/panda_urdf/panda.urdf')
    robot_description = {'robot_description': robot_description_config}
    robot_description_semantic_config = load_file(
        'ecard', 'config/moveit/panda_moveit_config/panda.srdf')
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config}
    kinematics_yaml = load_yaml(
        'ecard', 'config/moveit/panda_moveit_config/kinematics.yaml')
    robot_description_kinematics = {
        'robot_description_kinematics': kinematics_yaml}
    controllers_yaml = load_yaml('ecard', 'config/moveit/controllers.yaml')
    moveit_controllers = {'moveit_simple_controller_manager': controllers_yaml}
    ompl_planning_pipeline_config = {'ompl': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
        'start_state_max_bounds_error': 0.1}}
    ompl_planning_yaml = load_yaml(
        'ecard', 'config/moveit/panda_moveit_config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    rviz_config_file = get_package_share_directory(
        'ecard') + "/config/moveit/rviz2.rviz"

    return LaunchDescription([
        Node(package='tf2_ros',
             node_executable='static_transform_publisher',
             node_name='static_transform_publisher',
             output='log',
             arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'panda_link0']),

        Node(package='fake_joint_driver',
             node_executable='fake_joint_driver_node',
             parameters=[os.path.join(get_package_share_directory("ecard"), "config", "moveit", "panda_controllers.yaml"),
                         os.path.join(get_package_share_directory(
                             "ecard"), "config", "moveit", "start_positions.yaml"),
                         robot_description,
                         robot_description_kinematics]
             ),

        Node(node_name='ecard',
             package='ecard',
             node_executable='ecard',
             output='screen',
             parameters=[moveit_cpp_yaml_file_name,
                         robot_description,
                         robot_description_semantic,
                         kinematics_yaml,
                         ompl_planning_pipeline_config,
                         moveit_controllers]),

        Node(package='rviz2',
                     node_executable='rviz2',
                     node_name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description]),
    ])
