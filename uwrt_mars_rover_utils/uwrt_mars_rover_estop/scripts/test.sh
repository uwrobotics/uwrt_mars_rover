#!/bin/bash
while true; do
  #ITS RED!!!
  echo -e "\033[31mPress k to kill...\033[0m"
    #READS 1 CHARACTER FROM THE TERMINAL WHEN ENTER IS PRESSED AND STORES IT IN THE KEY VARAIBLE 
  read -n1 -r key
  if [ "$key" == $'k' ]; then
    ros2 topic pub /estop std_msgs/Bool "{data: true}" -1
  fi
done


# In a seperate terminal run 
# bash ~/uwrt_ws/src/uwrt_mars_rover/uwrt_mars_rover_utils/uwrt_mars_rover_estop/scripts/test.sh 

# ----------------------- FOR TESTING ---------------------------------------

#to build run: colcon build --packages-select uwrt_mars_rover_estop

#------------- HOW TO RUN IT SO YOU CAN SEE THE FUNCTIONALITY -------------------

#terminan1 : ros2 launch uwrt_mars_rover_estop estop_launch.py
#terminal2 : bash ~/uwrt_ws/src/uwrt_mars_rover/uwrt_mars_rover_utils/uwrt_mars_rover_estop/scripts/test.sh 
#terminal3 : ros2 topic pub /differential_drivetrain_controller/cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'

