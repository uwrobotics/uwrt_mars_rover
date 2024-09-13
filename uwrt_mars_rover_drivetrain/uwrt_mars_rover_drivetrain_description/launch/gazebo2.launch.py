import os
from ament_index_python import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler,TimerAction,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnExecutionComplete
from launch.substitutions import LaunchConfiguration, Command


from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'uwrt_mars_rover_drivetrain_description'
    file_subpath = 'urdf/drivetrain.urdf.xacro'
    controllers_config_path = get_package_share_path(
        'uwrt_mars_rover_drivetrain_hw') / 'config' / 'drivetrain_controllers.yaml'

    sim_status = LaunchConfiguration('use_sim')


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)

    robot_description_config = Command(['xacro ', xacro_file,' sim:=', 'true'])


    # Configure the node
    params={'robot_description': robot_description_config}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params] # add other parameters here if required
    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot'],
                    output='screen')
    

    differential_drivetrain_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'differential_drivetrain_controller'],
        output='screen'
    )

    joint_state_broadcaster_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )


    # Run the node
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Use ros2_control if true'),
       RegisterEventHandler(
        OnExecutionComplete(
            target_action=spawn_entity,
            on_completion=[
                TimerAction(
                    period=5.0,
                    actions=[joint_state_broadcaster_controller],
                )
            ]
        )
    ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_controller,
            on_exit=[differential_drivetrain_controller],
            )
        ),
        gazebo,
        spawn_entity,
        node_robot_state_publisher
        ])