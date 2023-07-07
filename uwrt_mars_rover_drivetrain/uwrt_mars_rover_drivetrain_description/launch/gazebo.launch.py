import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'uwrt_mars_rover_drivetrain_description'
    file_subpath = 'urdf/drivetrain.urdf.xacro'


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


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot'],
                    output='screen')

    # load_joint_state_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'joint_state_broadcaster'],
    #     output='screen'
    # )

    # load_tricycle_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'tricycle_controller'],
    #     output='screen'
    # )
    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=['ros2', 'control', 'load_controller', '--set-state', 'active',"differential_drivetrain_controller"],
    # )

    # joint_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=['ros2', 'control', 'load_controller', '--set-state', 'active',"joint_state_broadcaster"],
    # )
       # This helps to address race conditions on startup for the 
    # controller managers. Note that we are exporting delayed_controller_manager_spawner
    # instead of just diff_drive_spawner.
    
    # delayed_controller_manager_spawner = TimerAction(
    #     period=10.0,
    #     actions=[
    #         Node(
    #             package="controller_manager",
    #             executable="spawner",
    #             arguments=["differential_drivetrain_controller"],
    #         ),
    #         Node(
    #             package="controller_manager",
    #             executable="spawner",
    #             arguments=["joint_state_broadcaster"],
    #         )
    #     ],
    # )




    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        # diff_drive_spawner,
        # joint_broad_spawner
    ])