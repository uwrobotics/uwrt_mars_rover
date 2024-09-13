
# from http.server import executable
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package='joy',
        namespace="",
        executable="joy_node"
    )

    container = ComposableNodeContainer(
        name='container',
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="uwrt_mars_rover_xbox_controller",
                plugin="uwrt_xbox::UWRTXboxController"),
        ]
    )

    return launch.LaunchDescription([node, container])


"""
Process to launch the xbox control component
1. Build the uwrt_mars_rover_xbox_controller package (colcon build)
2. In a new terminal, source the overlay (source ~/ros2_ws/install/setup.bash)
3. Nagivate to the launch folder in the new terminal and run: ros2 launch xbox_launch.py

-> To see outputted results, see the /xbox_info topic
"""