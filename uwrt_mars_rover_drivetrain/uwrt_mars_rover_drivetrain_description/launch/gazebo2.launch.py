import os
from ament_index_python import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit 


from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'uwrt_mars_rover_drivetrain_description'
    file_subpath = 'urdf/drivetrain.urdf.xacro'
    controllers_config_path = get_package_share_path(
        'uwrt_mars_rover_drivetrain_hw') / 'config' / 'drivetrain_controllers.yaml'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )



    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot'],
                    output='screen')
    
    #loading in the controller manager
    controller_manager_launcher = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description_raw, controllers_config_path],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )


    differential_drivetrain_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'differential_drivetrain_controller'],
        output='screen'
    )

    joint_state_broadcaster_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )


    # Run the node
    return LaunchDescription([
        controller_manager_launcher,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_controller,
                on_exit=[differential_drivetrain_controller],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])