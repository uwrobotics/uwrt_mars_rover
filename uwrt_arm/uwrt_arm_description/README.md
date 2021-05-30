# uwrt_arm_description

This package contains the urdf, meshes and launch files for configuring the robot_description

## Launch Files

### description.launch

This loads the `robot_description` parameter into the parameter server by parsing the urdf, and launches the `robot_state_publisher` (to publish the TF tree), and optionally, the `joint_state_publisher` (to publish joint states). Note that if using `ros_control`, the controller manager is responsible for publish joint states and the `joint_state_publisher` should be disabled in that case.

#### Arguments

- `publish_joint_states`: Publish joint states using default ROS mechanism. **Default:** `true`
- `use_joint_states_gui`: Run joint states GUI for default ROS mechanism. **Default:** `false`
- `urdf_file`: The URDF file to parse for the `robot_description`. **Default:** `urdf/uwrt_arm.urdf.xacro`

## FAQs

1. The following type of errors may be observed when using the `launch/description.launch` launch file:

        [ERROR] [1549875853.096599133, 0.167000000]: No p gain specified for pid. Namespace: /gazebo_ros_control/pid_gains/<joint_name>

    The error is due to an upstream bug (https://github.com/ros-simulation/gazebo_ros_pkgs/issues/886) and it does not affect the operability in any way.
