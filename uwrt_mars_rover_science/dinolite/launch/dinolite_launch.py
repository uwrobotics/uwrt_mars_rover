import launch

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import (
    OnExecutionComplete,
    OnProcessExit,
    OnProcessIO,
    OnProcessStart,
    OnShutdown,
)
from launch.events import Shutdown
from launch.substitutions import (
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    LocalSubstitution,
    PythonExpression,
)


def generate_launch_description():
    cam_node_container = ComposableNodeContainer(
        name="cam_node_container",
        namespace="cam",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="dinolite", plugin="dinolite::CamNode", name="cam_node"
            )
        ],
        output="screen",
    )
    process_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=cam_node_container,
            on_exit=[
                LogInfo(
                    msg=(
                        EnvironmentVariable(name="USER"),
                        " closed the turtlesim window",
                    )
                ),
                EmitEvent(event=Shutdown(reason="Window closed")),
            ],
        )
    )
    shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                LogInfo(
                    msg=[
                        "Launch was asked to shutdown: ",
                        LocalSubstitution("event.reason"),
                    ]
                )
            ]
        )
    )

    return launch.LaunchDescription([cam_node_container, process_exit, shutdown])
