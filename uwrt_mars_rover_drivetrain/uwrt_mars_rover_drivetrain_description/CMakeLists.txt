cmake_minimum_required(VERSION 3.5)
project(uwrt_mars_rover_drivetrain_description)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_msgs
  geometry_msgs
  builtin_interfaces
  nav2_core
  pluginlib
)

install(
    DIRECTORY
      config
      launch
      rviz
      urdf
      world
    DESTINATION
      share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # flake8
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(
      CONFIG_FILE ${CMAKE_SOURCE_DIR}/../../.flake8
  )

  # xmllint
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif()

# for temporary standalone executable, for navigating to a given point
include_directories(include)
add_executable(execute_trajectory src/execute_trajectory.cpp include/uwrt_mars_rover_drivetrain_description/execute_trajectory.hpp)
ament_target_dependencies(execute_trajectory ${dependencies})
install(TARGETS
  execute_trajectory
  DESTINATION lib/${PROJECT_NAME})

  ament_export_include_directories(
    include
)

ament_package()
