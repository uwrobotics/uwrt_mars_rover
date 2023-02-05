cmake_minimum_required(VERSION 3.8)
project(dinolite)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Try for OpenCV 4.X, but settle for whatever is installed
find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif ()
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)

# Create ament index resource which references the libraries in the binary dir
set(node_plugins "")

#=============
# OpenCV camera node
#=============

add_library(
  cam_node SHARED
  src/cam_node.cpp
)
target_compile_definitions(
  cam_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(cam_node
  camera_calibration_parsers
  class_loader
  OpenCV
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_nodes(cam_node "dinolite::CamNode")
set(node_plugins "${node_plugins}dinolite::CamNode;$<TARGET_FILE:cam_node>\n")

#=============
# Test subscriber node
#=============

# add_library(
#   subscriber_node SHARED
#   src/subscriber_node.cpp
# )
# target_compile_definitions(
#   subscriber_node
#   PRIVATE "COMPOSITION_BUILDING_DLL"
# )
# ament_target_dependencies( subscriber_node
#   class_loader
#   rclcpp
#   rclcpp_components
#   sensor_msgs
# )
# rclcpp_components_register_nodes(subscriber_node "dinolite::ImageSubscriberNode")
# set(node_plugins "${node_plugins}dinolite::ImageSubscriberNode;$<TARGET_FILE:subscriber_node>\n")

# Install nodes
install(
  TARGETS cam_node
  EXPORT export_cam_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(
#   TARGETS subscriber_node
#   EXPORT export_subscriber_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# Install various directories
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

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

ament_export_dependencies(class_loader)

ament_export_include_directories(include)

ament_export_targets(export_cam_node)
# ament_export_targets(export_cam_node export_subscriber_node)

ament_export_libraries(gscam_node subscriber_node)
ament_package()


