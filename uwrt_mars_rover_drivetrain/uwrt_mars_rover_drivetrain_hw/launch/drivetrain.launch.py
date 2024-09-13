"""Launch Module for starting drivetrain on real hardware."""
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Generate launch description from list of nodes to launch.

    :return:
        LaunchDescription object
    """
    drivetrain_description_package_path = get_package_share_path('uwrt_mars_rover_drivetrain_description')
    model_path = drivetrain_description_package_path / 'urdf' / 'drivetrain.urdf.xacro'
    rviz_config_path = drivetrain_description_package_path / 'rviz' / 'urdf.rviz'
    controllers_config_path = get_package_share_path(
        'uwrt_mars_rover_drivetrain_hw') / 'config' / 'drivetrain_controllers.yaml'

    robot_description_content = ParameterValue(Command(['ros2 run xacro xacro ', str(model_path),' sim:=', 'false']), value_type=str)
    robot_description = {'robot_description': robot_description_content}

    # Nodes
    nodes = []
    nodes += [Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_config_path],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )]  # TODO: use custom control node w/ RT scheduling(port from ros1 uwrt_mars_rover branch)

    nodes += [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
        remappings=[
            ("/differential_drivetrain_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )]

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    nodes += [joint_state_broadcaster_spawner]

    # Delay rviz2 start after joint_state_broadcaster_spawner finishes
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(rviz_config_path)],
    )
    nodes += [RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )]

    # Delay start of drivetrain_controller_spawner after joint_state_broadcaster_spawner
    drivetrain_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['differential_drivetrain_controller', "--controller-manager", '/controller_manager'],
    )
    nodes += [RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[drivetrain_controller_spawner],
        )
    )]

    return LaunchDescription(nodes)
