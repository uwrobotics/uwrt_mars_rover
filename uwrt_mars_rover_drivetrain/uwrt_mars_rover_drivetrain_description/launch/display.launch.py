from ament_index_python.packages import get_package_prefix, get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_path = get_package_share_path('uwrt_mars_rover_drivetrain_description')
    default_model_path = package_path / 'urdf' / 'drivetrain.urdf.xacro'
    default_rviz_config_path = package_path / 'rviz' / 'urdf.rviz'

    robot_description_content = ParameterValue(Command([f'ros2 run xacro xacro ', LaunchConfiguration('model')]),
                                               value_type=str)

    # Declared Arguments
    declared_arguments = []
    declared_arguments += [DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                                 description='Flag to enable joint_state_publisher_gui')]
    declared_arguments += [DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                                 description='Absolute path to robot urdf file')]
    declared_arguments += [DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                                 description='Absolute path to rviz config file')]

    # Nodes
    nodes = []
    nodes += [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )]

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    nodes += [Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )]

    nodes += [Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )]

    nodes += [Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )]

    return LaunchDescription(declared_arguments + nodes)
