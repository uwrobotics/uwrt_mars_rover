#catkin_lint: ignore target_name_collision
cmake_minimum_required(VERSION 2.8.3)
project(uwrt_arm_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  roscpp
  uwrt_arm_msgs
  uwrt_arm_description
  uwrt_cartesian_controller
)

catkin_package(
  CATKIN_DEPENDS
    control_msgs
    uwrt_arm_msgs
    uwrt_arm_description
    uwrt_cartesian_controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
