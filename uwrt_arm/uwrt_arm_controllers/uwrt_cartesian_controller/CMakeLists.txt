#catkin_lint: ignore target_name_collision
cmake_minimum_required(VERSION 2.8.3)
project(uwrt_cartesian_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  control_toolbox
  controller_interface
  hardware_interface
  joint_trajectory_controller
  kdl_parser
  realtime_tools
  roscpp
  uwrt_arm_description
  uwrt_arm_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    control_msgs
    control_toolbox
    controller_interface
    hardware_interface
    kdl_parser
    realtime_tools
    roscpp
    uwrt_arm_description
    uwrt_arm_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(cartesian_controller
  include/uwrt_cartesian_controller/cartesian_controller.h
   src/cartesian_controller.cpp
)

target_link_libraries(cartesian_controller
  ${catkin_LIBRARIES}
)

install(
  FILES uwrt_cartesian_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
