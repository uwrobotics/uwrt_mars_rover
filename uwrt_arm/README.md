# uwrt_arm

Metapackage for the UWRT Arm project

## Running the Arm

### Real Arm

1. Attach CANable to computer

1. Setup and initialize CAN device

        sudo modprobe can
        sudo modprobe can_raw
        sudo modprobe can_dev
        sudo ip link set can0 type can bitrate 500000
        sudo ip link set up can0

1. Attach wired xbox 360 joystick to computer

1. Launch joystick node from the `joystick` package (optional). This publishes joystick commands to the `/arm_voltage_controller/command` topic. This node was created on the day before SAR by someone else and is super hacky.

        roslaunch joystick turtle_joy.launch

1. Bringup the arm code (description, ros controllers, moveit, etc)

        roslaunch uwrt_arm_bringup arm_bringup.launch can_device:=can0

1. Control the arm using the joystick

### Simulated Arm in Gazebo

1. Attach  wired xbox 360 joystick to computer

1. Bringup the arm code (description, moveit etc)

        roslaunch uwrt_arm_bringup arm_bringup.launch sim:=true

1. Launch gazebo simulation for arm. Gazebo also launches the ros controller interfaces which are loaded as a plugin

        roslaunch rover_simulation empty_world.launch

1. Currently, open loop voltage control is not supported in simulation, so the warning `[ArmHWSim] Voltage Control Mode not currently supported in simulation!` should be seen. Switch to velocity control by issuing a ROS service call

        rosservice call /controller_manager/switch_controller "start_controllers: ['arm_velocity_controller']
        stop_controllers: ['arm_voltage_controller']
        strictness: 0
        start_asap: false
        timeout: 0.0"

1. You should now be able to control each of the joints in velocity control!


