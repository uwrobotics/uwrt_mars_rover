"""Python launch file for running vectornav node."""

import launch
from launch.actions import (
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import (
    OnProcessExit,
    OnShutdown,
)
from launch.events import Shutdown
from launch.substitutions import (
    EnvironmentVariable,
    LocalSubstitution,
)

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate a launch description for vectornav node."""
    cam_node_container = ComposableNodeContainer(
        name='vectornav_node_container',
        namespace='vectornav',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='vectornav', plugin='vectornav::VectornavNode', name='vectornav_node'
            )
        ],
        output='screen',
    )
    process_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=vectornav_node_container,
            on_exit=[
                LogInfo(
                    msg=(
                        EnvironmentVariable(name='USER'),
                        ' killed process',
                    )
                ),
                EmitEvent(event=Shutdown(reason='Window closed')),
            ],
        )
    )
    shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                LogInfo(
                    msg=[
                        'Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason'),
                    ]
                )
            ]
        )
    )

    return launch.LaunchDescription([vectornav_node_container, process_exit, shutdown])
