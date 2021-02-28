cmake_minimum_required(VERSION 3.0.2)
project(led_matrix_control_sar)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  uwrt_mars_rover_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs
  uwrt_mars_rover_msgs
)

catkin_install_python(PROGRAMS
  scripts/set_led_matrix_server.py
  scripts/set_led_matrix_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
