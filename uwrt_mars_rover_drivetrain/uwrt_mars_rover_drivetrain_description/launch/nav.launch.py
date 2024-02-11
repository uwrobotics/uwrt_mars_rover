from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterFile

import os

def generate_launch_description():

    declared_arguments = []
    nodes = []

    controller_yaml = os.path.join(get_package_share_directory('uwrt_mars_rover_drivetrain_description'), 'config', 'costmap_parameters.yaml')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'map_server']
    
    nodes +=  [Node(
                package='nav2_map_server',
                executable='map_server',
                output='screen',
                parameters=[controller_yaml])]

    nodes += [Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[controller_yaml],
                remappings=[
                ("/cmd_vel", "/differential_drivetrain_controller/cmd_vel_unstamped")])]
    
    nodes += [Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[controller_yaml])]

    nodes += [Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'autostart': True},
                            {'node_names': lifecycle_nodes}])]

    return LaunchDescription(declared_arguments + nodes)
