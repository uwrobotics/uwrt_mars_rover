cmake_minimum_required(VERSION 2.8.3)
project(uwrt_arm_workspace)

add_compile_options(-std=c++11)

# Find the Eigen3 libraries
find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
  # Just change the name of the eigen variables
  # to the expectation of the catkin build system
  set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
  set(Eigen3_LIBRARIES    ${EIGEN3_LIBRARIES})
endif()

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  rviz_visual_tools
  pcl_conversions
  pcl_ros
)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    rospy
    tf2
    tf2_ros
    visualization_msgs
    rviz_visual_tools
  DEPENDS
    Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(publish_workspace_markers
  src/publish_workspace_markers.cpp
)

target_link_libraries(publish_workspace_markers
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(
  TARGETS publish_workspace_markers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(publish_workspace_mesh
  src/publish_workspace_mesh.cpp
)

target_link_libraries(publish_workspace_mesh
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(
  TARGETS publish_workspace_mesh
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY cfg launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} 
)
