cmake_minimum_required(VERSION 2.8.3)
project(uwrt_mars_rover_hw_bridge)
find_package(catkin REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ar_tracker
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_subdirectory(library/${PROJECT_NAME})
