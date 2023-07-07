import os
from ament_index_python import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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


    #This stores the information of the the robots joints etc
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


    #launching gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )
    
    #----------------------------------------------------------------------------------


    drivetrain_description_package_path = get_package_share_path('uwrt_mars_rover_drivetrain_description')
    model_path = drivetrain_description_package_path / 'urdf' / 'drivetrain.urdf.xacro'
    rviz_config_path = drivetrain_description_package_path / 'rviz' / 'urdf.rviz'
    controllers_config_path = get_package_share_path(
        'uwrt_mars_rover_drivetrain_hw') / 'config' / 'drivetrain_controllers.yaml'

    robot_description_content = ParameterValue(Command(['ros2 run xacro xacro ', str(model_path)]), value_type=str)
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
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}],
        remappings=[
            ("/differential_drivetrain_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )]

    nodes += [joint_state_broadcaster_spawner := Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )]

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

    nodes += [gazebo]
    #----------------------------------------------------------------------------------







    # Run the node
    return LaunchDescription(
        nodes
    )