from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uwrt_mars_rover_led_matrix',
            namespace='rgb_pin_node',
            executable='led_matrix',
            name='rgb_pwm_pin_node'
        ),
    ])