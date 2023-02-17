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
    nodes = []

    nodes += [Node(
        package='uwrt_mars_rover_utilities',
        executable='uwrt_can_test_node',
        parameters=[
        {"CAN_interface":"can0"}],
        namespace='',
        name='can_test_node'
    )]  

    return LaunchDescription(nodes)
