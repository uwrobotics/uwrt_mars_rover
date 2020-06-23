#catkin_lint: ignore target_name_collision
cmake_minimum_required(VERSION 2.8.3)
project(uwrt_cartesian_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  roscpp
  roslint
  uwrt_arm_msgs
  uwrt_arm_description
  kdl_parser
  control_toolbox
  controller_interface
  hardware_interface
  realtime_tools
  joint_trajectory_controller
)

find_package(Eigen3 REQUIRED)

# Roslint
set(ROSLINT_CPP_OPTS "--filter=-build/header_guard,-runtime/references")
roslint_cpp()

catkin_package(
  CATKIN_DEPENDS
    uwrt_arm_msgs
    uwrt_arm_description
    control_msgs
    roscpp
    control_toolbox
    controller_interface
    hardware_interface
    realtime_tools
    kdl_parser
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

