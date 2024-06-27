from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterFile

import os

def generate_launch_description():

    declared_arguments = [
        # TODO: add this back after fixing the map path (if we need a static map) with the USER env var (if needed)
        # DeclareLaunchArgument(
        #     "map_yaml_filename",
        #     default_value=os.path.join(get_package_share_directory('uwrt_mars_rover_drivetrain_description'), 'config', 'map.yaml'),
        #     description='Map yaml file path'
        # )
    ]

    ekf_launch_file = os.path.join(
        get_package_share_directory('uwrt_mars_rover_drivetrain_description'),
        'launch',
        'dual_ekf_navsat.launch.py'
    )

    ekf_launch = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            ekf_launch_file
        )
    )]

    nodes = []

    nav_config_yaml = os.path.join(get_package_share_directory('uwrt_mars_rover_drivetrain_description'), 'config', 'costmap_parameters.yaml')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
    ]
    # TODO: add this back after fixing the map path with the USER env var
                    #    'map_server']
    
    # nodes +=  [Node(
    #             package='nav2_map_server',
    #             executable='map_server',
    #             output='screen',
    #             # parameters=[{"yaml_file": LaunchConfiguration("map_yaml_filename")}, nav_config_yaml],
    #             parameters=[nav_config_yaml]
    #             )]

    nodes += [Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[nav_config_yaml])]
    
    nodes += [Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[nav_config_yaml])]

    nodes += [Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'autostart': True},
                            {'node_names': lifecycle_nodes}])]

    return LaunchDescription(ekf_launch + declared_arguments + nodes)
