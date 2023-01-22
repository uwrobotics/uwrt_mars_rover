"""
Launch Module for visualizing drivetrain in rviz.

Allows for manipulation of joints via joint_state_publisher_gui if gui arg is set.
"""
from ament_index_python.packages import get_package_share_path, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    """
    Generate launch description from launch arguments and list of nodes to launch.

    :return:
        LaunchDescription object
    """
    package_path = get_package_share_path('uwrt_mars_rover_drivetrain_description')
    default_model_path = package_path / 'urdf' / 'drivetrain.urdf.xacro'
    default_rviz_config_path = package_path / 'rviz' / 'urdf.rviz'

    robot_description_content = ParameterValue(Command(['ros2 run xacro xacro ', LaunchConfiguration('model')]),
                                               value_type=str)

    # Declared Arguments
    declared_arguments = []
    declared_arguments += [DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                                 description='Flag to enable joint_state_publisher_gui')]
    declared_arguments += [DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                                 description='Absolute path to robot urdf file')]
    declared_arguments += [DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                                 description='Absolute path to rviz config file')]
    declared_arguments+=[DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                                 description='Use simulation (Gazebo) clock if true')]
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Nodes
    nodes = []
    nodes += [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
         output='screen',
    )]

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    nodes += [Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )]

    nodes += [Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )]

    nodes += [Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )]
    gazebo_starter = [ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen')]
    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    #     )
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py', name='urdf_spawner',
                        arguments=['-topic', 'robot_description', '-entity', 'drivetrain', '-x', '0', '-y', '0', '-z', '0'], output='screen'
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'differential_drivetrain_controller'],
        output='screen'
    )
    loadjointstate =  RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        )
    loadtrajectorycontroller =  RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        )
    ros2_control_nodes = [loadjointstate,loadtrajectorycontroller]

    nodes+=gazebo_starter + [spawn_entity]


    return LaunchDescription(declared_arguments + nodes + ros2_control_nodes)
