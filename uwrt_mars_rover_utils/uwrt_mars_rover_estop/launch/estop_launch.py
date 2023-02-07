import launch
import launch_ros.actions
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)



def generate_launch_description():


        #-------------------------------container for turtle_clear-----------------------------------
        
    estop_container = ComposableNodeContainer(
            name='node_container1',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                    ComposableNode(
                            package='uwrt_mars_rover_estop',
                            plugin='composition::estop',
                            name='estop_node',
                    )                      

            ]
    )
    

    
    #-------------------------------------------------------------------------------
    return launch.LaunchDescription([estop_container])