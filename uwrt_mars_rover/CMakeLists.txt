cmake_minimum_required(VERSION 3.14.4)
project(uwrt_mars_rover)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  # xmllint
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif()

ament_package()
