#catkin_lint: ignore literal_project_name
cmake_minimum_required(VERSION 2.8.3)
project(uwrt_arm_hw)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    angles
    controller_manager
    gazebo_dev
    gazebo_ros_control
    hardware_interface
    joint_limits_interface
    roscpp
    roslint
    transmission_interface
    urdf
)

find_package(GAZEBO REQUIRED)

# Roslint
set(ROSLINT_CPP_OPTS "--filter=-build/header_guard,-runtime/references")
roslint_cpp()

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    gazebo_${PROJECT_NAME}
  CATKIN_DEPENDS
    angles
    controller_manager
    gazebo_ros_control
    hardware_interface
    joint_limits_interface
    roscpp
    transmission_interface
    urdf
  DEPENDS GAZEBO
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Compile the Gazebo Plugin
add_library(gazebo_${PROJECT_NAME} SHARED
  src/arm_control_loop.cpp
  src/arm_hw.cpp
  src/arm_hw_sim.cpp
  src/gazebo_arm_control_plugin.cpp
)

target_link_libraries(gazebo_${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Compile the Real Arm HW Interface Node
add_executable(${PROJECT_NAME}_real
  src/arm_control_loop.cpp
  src/arm_hw.cpp
  src/arm_hw_real.cpp
  src/arm_hw_real_node.cpp
  src/real_arm_control.cpp
)

target_link_libraries(${PROJECT_NAME}_real
  ${catkin_LIBRARIES}
)

install(
  TARGETS gazebo_${PROJECT_NAME} ${PROJECT_NAME}_real
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
