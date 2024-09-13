import os
import launch
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    container = ComposableNodeContainer(
        name='container',
        namespace="",
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='uwrt_mars_rover_xbox_controller',
                plugin='drivetraincontrollerComposition::CoordinateNode',
                name='coordinateNode'
            ),
        ],
        output='screen'
    )
    
    return launch.LaunchDescription([container])
