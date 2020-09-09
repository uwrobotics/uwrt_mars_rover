#catkin_lint: ignore literal_project_name
cmake_minimum_required(VERSION 2.8.3)
project(uwrt_arm_hw)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    angles
    controller_interface
    controller_manager
    forward_command_controller
    gazebo_dev
    gazebo_ros_control
    gripper_action_controller
    hardware_interface
    joint_limits_interface
    pluginlib
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
    ${PROJECT_NAME}_voltage_controllers
    gazebo_${PROJECT_NAME}
  CATKIN_DEPENDS
    angles
    controller_interface
    controller_manager
    forward_command_controller
    gazebo_ros_control
    gripper_action_controller
    hardware_interface
    joint_limits_interface
    pluginlib
    roscpp
    transmission_interface
    urdf
  DEPENDS GAZEBO
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_voltage_controllers
  src/joint_group_voltage_controller.cpp
  src/joint_voltage_controller.cpp
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
  ${PROJECT_NAME}_voltage_controllers
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
  ${PROJECT_NAME}_voltage_controllers
)

install(
  TARGETS
    gazebo_${PROJECT_NAME}
    ${PROJECT_NAME}_real
    ${PROJECT_NAME}_voltage_controllers
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

install(FILES voltage_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
