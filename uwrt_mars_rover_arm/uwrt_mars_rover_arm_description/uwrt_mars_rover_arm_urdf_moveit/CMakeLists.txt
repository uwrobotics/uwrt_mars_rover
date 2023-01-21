cmake_minimum_required(VERSION 3.10.2)
project(uwrt_mars_rover_arm_urdf_moveit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

if (BUILD_TESTING)
  # generate compile_commands.json for clang-tidy
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

  # cppcheck
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck()

  # clang-format
  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format()

  # clang-tidy
  find_package(ament_cmake_clang_tidy REQUIRED)
  ament_clang_tidy(${CMAKE_BINARY_DIR})

  # flake8
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()

  # xmllint
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif ()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MotionPlan.srv"
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

add_library(motion_plan_server SHARED
  src/motion_plan_server.cpp)
ament_target_dependencies(motion_plan_server
  "rclcpp"
  "geometry_msgs"
  "moveit_ros_planning_interface")

rosidl_target_interfaces(motion_plan_server ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(motion_plan_server "uwrt_motion_planning::MotionPlanServer")
set(node_plugins "${node_plugins}uwrt_motion_planning::MotionPlanServer;$<TARGET_FILE:motion_plan_server>\n")

add_library(motion_plan_client SHARED
  src/motion_plan_client.cpp)
ament_target_dependencies(motion_plan_client
  "rclcpp"
  "geometry_msgs"
  "moveit_ros_planning_interface")

rosidl_target_interfaces(motion_plan_client ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(motion_plan_client "uwrt_motion_planning::MotionPlanClient")
set(node_plugins "${node_plugins}uwrt_motion_planning::MotionPlanClient;$<TARGET_FILE:motion_plan_client>\n")

install(TARGETS motion_plan_server motion_plan_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib 
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})


ament_package()
