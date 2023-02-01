cmake_minimum_required(VERSION 3.5)
project(uwrt_mars_rover_vision)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

#include the 'include' directory
include_directories(include)

#create resource which references the libraries in the binary bin
set(node_plugins "")

# add the aruco target tracker component
add_library(aruco_tracker SHARED 
	src/aruco_target_tracker.cpp)
target_compile_definitions(aruco_tracker
	PRIVATE "UWRT_MARS_ROVER_VISION_DLL")
ament_target_dependencies(aruco_tracker
	"rclcpp"
	"rclcpp_components"
	"sensor_msgs"
	"cv_bridge"
	"image_transport"
	"geometry_msgs"
	"OpenCV"
	)
rclcpp_components_register_nodes(aruco_tracker "uwrt_autonomy::TargetTracker")
set(node_plugins "${node_plugins}uwrt_autonomy::TargetTracker;$<TARGET_FILE:aruco_tracker>\n")

# tell where to put binaries
install(TARGETS
  aruco_tracker
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib 
	RUNTIME DESTINATION bin)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
