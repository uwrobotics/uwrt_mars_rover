cmake_minimum_required(VERSION 3.5)
project(uwrt_arm_cartesian_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(realtime_tools REQUIRED)


add_library(uwrt_arm_cartesian_controller
  SHARED
  src/uwrt_arm_cartesian_controller.cpp
)
target_include_directories(uwrt_arm_cartesian_controller PRIVATE include)
ament_target_dependencies(uwrt_arm_cartesian_controller
  builtin_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  geometry_msgs
  orocos_kdl
  realtime_tools
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(uwrt_arm_cartesian_controller PRIVATE "UWRT_ARM_CARTESIAN_CONTROLLER_BUILDING_LIBRARY")
pluginlib_export_plugin_description_file(controller_interface uwrt_arm_cartesian_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  uwrt_arm_cartesian_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(FALSE) # younes todo add unit testing like in the forwarding controller example BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
    test_load_uwrt_arm_cartesian_controller
    test/test_load_uwrt_arm_cartesian_controller.cpp
  )
  target_include_directories(test_load_uwrt_arm_cartesian_controller PRIVATE include)
  ament_target_dependencies(test_load_uwrt_arm_cartesian_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(
    test_uwrt_arm_cartesian_controller
    test/test_uwrt_arm_cartesian_controller.cpp
  )
  target_include_directories(test_uwrt_arm_cartesian_controller PRIVATE include)
  target_link_libraries(test_uwrt_arm_cartesian_controller
    uwrt_arm_cartesian_controller
  )
endif()

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

ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  orocos_kdl
  realtime_tools
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  uwrt_arm_cartesian_controller
)
ament_package()