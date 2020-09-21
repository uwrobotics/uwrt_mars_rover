#catkin_lint: ignore target_name_collision
cmake_minimum_required(VERSION 2.8.3)
project(uwrt_arm_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  roscpp
  uwrt_arm_description
  uwrt_arm_msgs
  uwrt_cartesian_controller
)

catkin_package(
  CATKIN_DEPENDS
    control_msgs
    uwrt_arm_description
    uwrt_arm_msgs
    uwrt_cartesian_controller
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

