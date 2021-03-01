cmake_minimum_required(VERSION 2.8.3)
project(drive_to_gps)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  actionlib
  uwrt_mars_rover_msgs
  uwrt_mars_rover_utils
  gazebo_ros
)

##find_package(Boost REQUIRED COMPONENTS system)

find_package(gazebo REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/CurrentHeading.cfg
)

catkin_package(
  DEPENDS gazebo_ros
  INCLUDE_DIRS include
  LIBRARIES drive_to_gps
  CATKIN_DEPENDS
  uwrt_mars_rover_utils
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/utils.cpp
  ##src/calculate_twist_server.cpp
  ##src/current_heading.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(current_heading src/current_heading.cpp)
target_link_libraries(current_heading ${catkin_LIBRARIES} drive_to_gps)

add_executable(calculate_twist_server src/calculate_twist_server.cpp)

add_executable(gps_test src/gps_test.cpp)
target_link_libraries(gps_test ${catkin_LIBRARIES})

target_link_libraries(
  calculate_twist_server
  drive_to_gps
  ${catkin_LIBRARIES}
)

add_dependencies(
  calculate_twist_server
  ${drive_to_gps_EXPORTED_TARGETS}
)