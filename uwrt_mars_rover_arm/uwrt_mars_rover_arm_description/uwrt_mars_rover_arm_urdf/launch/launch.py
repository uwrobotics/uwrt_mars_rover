from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='uwrt_mars_rover_arm_urdf',
            description='package',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='urdf.xacro',
            description='urdf file',
        )
    )

    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'urdf', description_file]),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), 'rviz', 'rviz.rviz']
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
