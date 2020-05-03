
# Improvements

1. Use `combined_robot_hw` for Arm hardware interfaces. This is needed so it will play nicely with the other hardware interfaces used by the drivetrain, science module etc.

1. Implement `ros_control` interface for the gripper action. Currently, all the primary arm joints are controlled using proper `ros_control` interfaces i.e. `joint_state_controller`, `position_controllers/JointGroupPositionController`, `velocity_controllers/JointGroupVelocityController`, `voltage_controllers/JointGroupVoltageController`. Note that the `voltage_controllers/JointGroupVoltageController` is a custom `ros_control` interface that we created that follows the `ros_control` spec, for open-loop control of the arm. There exists a `gripper_action_controller` for controlling grippers using the `ros_control` framework which needs to be implemented (refer to [gripper_action_controller](https://github.com/ros-controls/ros_controllers/tree/melodic-devel/gripper_action_controller)). For SAR, we did the claw operation by directly sending the CAN frame from the command line

1. The Open Loop Voltage Control is currently not supported in simulation, only for the real robot, shouldn't be too much work to enable it for simulation too.

1. The CAN feedback code needs to be tested and might potentially need minor fixes. There were issues with getting sane encoder feedback from the joints before, and could not be tested.

1. Test velocity and position control with the real robot. Could not do this previously because of a lack of position and velocity feedback (as mentioned above).

# Issues

1. **Needs to be verified**. CAN reads blocking control loops indefinitely. This happened sometimes and not other times, and there were numerous changes to the firmware during testing, and so don't know if the issue still exists and the exact reason for the issue. Likely requires fairly minor fixes.

1. Cartesian control for jogging arm along planes (x,y,z) using position control was previously attempted but didn't work well. A likely cause of the issue is that the arm is not a 6-DOF arm like most commercial arms, and so incremental jogging along planes might not be possible (could be an ill-posed problem). 