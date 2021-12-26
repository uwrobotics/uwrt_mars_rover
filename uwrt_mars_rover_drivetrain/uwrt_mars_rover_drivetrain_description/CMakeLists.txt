cmake_minimum_required(VERSION 3.5)
project(uwrt_mars_rover_drivetrain_description)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY
      launch
      ros2_control
      rviz
      urdf
    DESTINATION
      share/${PROJECT_NAME}
)

ament_package()
