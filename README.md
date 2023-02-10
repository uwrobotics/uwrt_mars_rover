# uwrt_mars_rover_estop

##  Description
This node will check if the `ENTER` key was pressed in the script that is running in another terminal. If the enter key is pressed the node will         start publishing a velocity of 0 to the `stop_vel_interceptor` topic, this is the new topic that the drivetrain will subscribe to. If there was no stop 
requested the node will just keep publishing the values from `/differential_drivetrain_controller/cmd_vel` to the `estop_vel_interceptor` topic.


  
### How to setup to test functionality 

This is the setup to run it just for now because the `/differential_drivetrain_controller/cmd_vel` topic is not working.
In reality we will just need 1 terminal just to run the script.

1. To build Run: `colcon build --packages-select uwrt_mars_rover_estop` (Note: your ws might have a different name)
2. Terminal 1 : `ros2 launch uwrt_mars_rover_estop estop_launch.py`
3. Terminal 2 : `bash ~/uwrt_ws/src/uwrt_mars_rover/uwrt_mars_rover_utils/uwrt_mars_rover_estop/scripts/test.sh` 
4. Terminal 3 : `ros2 topic pub /differential_drivetrain_controller/cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'`
