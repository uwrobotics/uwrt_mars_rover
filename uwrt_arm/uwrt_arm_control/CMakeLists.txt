#catkin_lint: ignore target_name_collision
cmake_minimum_required(VERSION 2.8.3)
project(uwrt_arm_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  roscpp
  roslint
  uwrt_arm_msgs
)

# Roslint
set(ROSLINT_CPP_OPTS "--filter=-build/header_guard,-runtime/references")
roslint_cpp()

catkin_package(
  CATKIN_DEPENDS
    control_msgs
    uwrt_arm_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(delta_joint_publisher_node
  src/delta_joint_publisher.cpp
  src/delta_joint_publisher_node.cpp
)

target_link_libraries(delta_joint_publisher_node
  ${catkin_LIBRARIES}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS delta_joint_publisher_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
