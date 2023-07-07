import os
from ament_index_python import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch_ros.actions import Node
import xacro
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'uwrt_mars_rover_drivetrain_description'
    file_subpath = 'urdf/drivetrain.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    controllers_config_path = get_package_share_path(
        'uwrt_mars_rover_drivetrain_hw') / 'config' / 'drivetrain_controllers.yaml'


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}], # add other parameters here if required
        remappings=[
            ("/differential_drivetrain_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )



    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot'],
                    output='screen')


    nodes = []
    nodes += [Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description_raw, controllers_config_path],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )] 

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(rviz_config_path)],
    )


    nodes += [joint_state_broadcaster_spawner := Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )]

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




    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        # diff_drive_spawner,
        # joint_broad_spawner
    ])