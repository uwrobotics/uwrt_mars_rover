import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():
    container = ComposableNodeContainer(
        name='container',
        namespace="",
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package="uwrt_mars_rover",
                plugin="xboxcontrollerComposition::CoordinateNode"
            ),
        ]
    )
return launch.LaunchDescription([container])