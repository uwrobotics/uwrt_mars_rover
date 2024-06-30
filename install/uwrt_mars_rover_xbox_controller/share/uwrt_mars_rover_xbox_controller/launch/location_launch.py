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
                package='uwrt_mars_rover_xbox_controller',
                plugin='drivetraincontrollerComposition::CoordinateNode',
                name='coordinateNode'
            ),
        ]
    )
    return launch.LaunchDescription([container])
'''
import launch
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
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
                package='uwrt_mars_rover_xbox_controller',
                plugin='drivetraincontrollerComposition::CoordinateNode',
                name='coordinateNode'
            ),
        ]
    )
    
    return launch.LaunchDescription([
        # Set the ROS_IP environment variable
        SetEnvironmentVariable('ROS_IP', '10.0.0.1'),  # Replace with your desired IP address

        # Set other environment variables for Fast DDS
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        #SetEnvironmentVariable('RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),
        #SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', 'uwrt_mars_rover_xbox_controller/config/fastdds_profiles.xml'),  # Path to your XML file

        container
    ])
    '''
