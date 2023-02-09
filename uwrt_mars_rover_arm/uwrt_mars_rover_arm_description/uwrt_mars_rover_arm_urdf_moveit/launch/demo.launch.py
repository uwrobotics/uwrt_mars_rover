import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node

import xacro

import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    # planning_context
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory('uwrt_mars_rover_arm_urdf_moveit'),
            'config',
            'uwrt_mars_rover_arm_urdf.urdf.xacro',
        )
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        'uwrt_mars_rover_arm_urdf_moveit', 'config/uwrt_mars_rover_arm_urdf.srdf'
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        'uwrt_mars_rover_arm_urdf_moveit', 'config/kinematics.yaml'
    )

    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 respawn=True,
                                 output='screen',
                                 parameters=[
                                     robot_description
                                 ]
                                 )

    rviz_base = os.path.join(
        get_package_share_directory('uwrt_mars_rover_arm_urdf_moveit'), 'config')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
    )

    return LaunchDescription(
        [
            rviz_node,
            robot_state_publisher,
        ]
    )
