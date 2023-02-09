
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'controllers_file',
            default_value='arm_controllers.yaml',
            description='YAML file with the controllers configuration.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='uwrt_mars_rover_arm_description',
            description='Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            description='URDF/XACRO description file with the robot.',
            default_value='urdf.xacro'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Start robot in Gazebo simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Start robot with fake hardware mirroring command to its states.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'fake_sensor_commands',
            default_value='true',
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'slowdown', default_value='3.0', description='Slowdown factor of the robot.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_controller',
            default_value='forward_position_controller',
            description='Robot controller to start.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )

    # Initialize Arguments
    controllers_file = LaunchConfiguration('controllers_file')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    slowdown = LaunchConfiguration('slowdown')
    robot_controller = LaunchConfiguration('robot_controller')
    start_rviz = LaunchConfiguration('start_rviz')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'urdf', description_file]
            ),
            ' ',
            'use_sim:=',
            use_sim,
            ' ',
            'use_fake_hardware:=',
            use_fake_hardware,
            ' ',
            'fake_sensor_commands:=',
            fake_sensor_commands,
            ' ',
            'slowdown:=',
            slowdown,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(description_package),
         'config', controllers_file,
         ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), 'rviz', 'rviz.rviz']
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )
    robot_state_pub_node = Node(
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
        condition=IfCondition(start_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[robot_controller, '-c', '/controller_manager'],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    nodes = [
        gazebo,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
