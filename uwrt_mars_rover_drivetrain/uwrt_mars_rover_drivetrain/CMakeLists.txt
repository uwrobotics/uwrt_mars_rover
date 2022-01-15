cmake_minimum_required(VERSION 3.5)
project(uwrt_mars_rover_drivetrain)

find_package(ament_cmake REQUIRED)

if (BUILD_TESTING)
  # xmllint
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif ()

ament_package()
