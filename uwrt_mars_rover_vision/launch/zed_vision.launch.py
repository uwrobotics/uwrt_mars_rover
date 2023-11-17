from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch/zed2.launch.py'
                ])
            ])
        )

    container = ComposableNodeContainer(
        name='container',
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="uwrt_mars_rover_vision",
                plugin="uwrt_autonomy::TargetTracker",
                parameters=[
                    # marker dimension in meters
                    {"aruco_marker_len": 0.184 },
                    # aruco dictionary following the urc guidlines (boolean) 
                    {"aruco_dict": "4x4_50"},
                    # whether or not to show marker poses on the screen
                    {"display_marker_pose": True}
                    ]
                )
        ]
    )

    return LaunchDescription([zed_launch, container])


"""
Process to launch the target tracker component
1. Build the uwrt_mars_rover_vision package and zed_wrapper (colcon build)
2. In a new terminal, source the overlay (source ~/ros2_ws/install/setup.bash)
3. Nagivate to the launch folder in the new terminal and run: ros2 launch uwrt_mars_rover_vision zed_vision.launch.py
"""