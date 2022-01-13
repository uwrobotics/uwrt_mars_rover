from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from pathlib import Path


def generate_launch_description():

    robo = (
        Path(
            r"/home/niiquaye/robotics/install/uwrt_mars_rover_drivetrain_description/share/uwrt_mars_rover_drivetrain_description/urdf/robot.urdf"
        )
        .resolve(strict=True)
        .read_text()
    )

    robot_description = {"robot_description": robo}

    drivetrain_differential_drive_controller = PathJoinSubstitution(
        [FindPackageShare("uwrt_mars_rover_drivetrain_hw"), "config", "drivetrain_controller.yaml"]
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, drivetrain_differential_drive_controller],
        output={"stdout": "screen", "stderr": "screen"},
    )

    spawn_diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["drivetrain_differential_controller"],
        output="screen",
    )

    start_sync_slam_toolbox_node = Node(
        parameters=[
            get_package_share_directory("slam_toolbox") + "/config/mapper_params_online_sync.yaml",
            {"use_sim_time": use_sim_time},
        ],
        package="controller_manager",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        prefix=["xterm -e gdb -ex run --args"],
        output="screen",
    )

    return LaunchDescription([controller_manager_node, spawn_diff_drive_controller])

