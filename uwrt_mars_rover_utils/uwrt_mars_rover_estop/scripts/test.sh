#!/bin/bash
while true; do
  echo -e "\033[31mPress ENTER to kill...\033[0m"
  read -n1 -r key
  ros2 topic pub /estop std_msgs/String "{data: '$key'}" -1
done


# In a seperate terminal run 
# bash ~/uwrt_ws/src/uwrt_mars_rover/uwrt_mars_rover_utils/uwrt_mars_rover_estop/scripts/test.sh 