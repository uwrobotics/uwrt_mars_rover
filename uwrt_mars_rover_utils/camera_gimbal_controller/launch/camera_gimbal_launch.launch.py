from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_gimbal_controller',
            namespace='node1',
            executable='talker',
            name='talker_node'
        ),
        Node(
            package='camera_gimbal_controller',
            namespace='node2',
            executable='listener',
            name='listener_node'
        ),

    ])
