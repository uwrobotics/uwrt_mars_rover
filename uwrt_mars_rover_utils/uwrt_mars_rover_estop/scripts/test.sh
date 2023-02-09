#!/bin/bash
while true; do
#   This line reads a single character from the keyboard and stores it in the key variable. 
#   The -n1 option specifies to read only one character, 
#   the -r option disables backslash interpretation, 
#   and the -p option is used to display the message Press any key to continue... on the terminal before reading the key.
#   we store the 
  read -n1 -r -p "Press any key to continue..." key
  echo $key
  ros2 topic pub /estop std_msgs/String "{data: '$key'}" -1
done



# In a seperate terminal run 
# bash ~/uwrt_ws/src/uwrt_mars_rover/uwrt_mars_rover_utils/uwrt_mars_rover_estop/scripts/test.sh 