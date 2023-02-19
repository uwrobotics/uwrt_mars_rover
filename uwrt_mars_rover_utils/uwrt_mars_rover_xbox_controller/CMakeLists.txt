cmake_minimum_required(VERSION 3.5)
project(uwrt_mars_rover_xbox_controller)

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
# add custom messages and services
find_package(rosidl_default_generators REQUIRED)
# add ros joy package
find_package(sensor_msgs REQUIRED)

set(msg_files
	"msg/XboxController.msg"	
)

rosidl_generate_interfaces(${PROJECT_NAME}
	${msg_files}
	DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

#include the 'include' directory
include_directories(include)

#create resource which references the libraries in the binary bin
set(node_plugins "")

# add the xbox controller component
add_library(xbox_controller SHARED 
	src/uwrt_mars_rover_xbox_controller.cpp)
target_compile_definitions(xbox_controller
	PRIVATE "UWRT_MARS_ROVER_XBOX_CONTROLLER_DLL")
ament_target_dependencies(xbox_controller
	"rclcpp"
	"rclcpp_components"
	"sensor_msgs")
# build ROS custom messages
rosidl_target_interfaces(xbox_controller ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(xbox_controller "uwrt_xbox::UWRTXboxController")
set(node_plugins "${node_plugins}uwrt_xbox::UWRTXboxController;$<TARGET_FILE:xbox_controller>\n")

# add the node for coordinates
add_library(coordinateNode SHARED 
	src/coordinateNode.cpp)
target_compile_definitions(coordinateNode
	PRIVATE "COORDINATE_NODE_DLL")
ament_target_dependencies(coordinateNode
	"rclcpp"
	"rclcpp_components")
# build ROS custom messages
rosidl_target_interfaces(coordinateNode ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(coordinateNode "drivetraincontrollerComposition::coordinateNode")
set(node_plugins "${node_plugins}drivetraincontrollerComposition::coordinateNode;$<TARGET_FILE:coordinateNode>\n")


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

install(DIRECTORY launch
		DESTINATION share/${PROJECT_NAME}
)

# tell where to put binaries
install(TARGETS
  xbox_controller
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib 
	RUNTIME DESTINATION bin)

if (BUILD_TESTING)
   # Force generation of compile_commands.json for clang-tidy
   set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

   # clang-format
   find_package(ament_cmake_clang_format REQUIRED)
   ament_clang_format(
   CONFIG_FILE ${CMAKE_SOURCE_DIR}/../../.clang-format
   )

   # clang-tidy
   find_package(ament_cmake_clang_tidy REQUIRED)
   ament_clang_tidy(
   ${CMAKE_BINARY_DIR}
   CONFIG_FILE ${CMAKE_SOURCE_DIR}/../../.clang-tidy
   )

   # cppcheck
   find_package(ament_cmake_cppcheck REQUIRED)
   ament_cppcheck()

   # flake8
   find_package(ament_cmake_flake8 REQUIRED)
   ament_flake8(
   CONFIG_FILE ${CMAKE_SOURCE_DIR}/../../.flake8
   )

   # xmllint
   find_package(ament_cmake_xmllint REQUIRED)
   ament_xmllint()
endif ()

ament_package()