cmake_minimum_required(VERSION 3.8)
project(uwrt_mars_rover_estop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)




include_directories(include)
set(node_plugins "")


#add kill_all_turtle_service 
add_library(estop_node SHARED 
	src/estop.cpp)
target_compile_definitions(estop_node
	PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(estop_node
	"rclcpp"
	"rclcpp_components"
  	"std_msgs"
  	"geometry_msgs")
rclcpp_components_register_nodes(estop_node "uwrt_mars_rover_estop::Estop")
# this way we can execute the component with - ros2 component standalone software_training composition::turtle_service_request_node
set(node_plugins "${node_plugins}uwrt_mars_rover_estop::Estop;$<TARGET_FILE:estop_node>\n")




if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()


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
endif()


#tell where to put binaries
install(TARGETS
	estop_node
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib 
	RUNTIME DESTINATION bin)

#install launch file
install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
	)



ament_package()
