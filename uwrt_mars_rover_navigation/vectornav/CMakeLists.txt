cmake_minimum_required(VERSION 3.8)
project(vectornav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# vncxx
add_subdirectory(vnproglib-1.2.0.0/cpp)

# simplify includes
include_directories(
  include
)

# vectornav_node
add_library( ${PROJECT_NAME}
  SHARED src/vectornav_node.cpp
)
# target_compile_definitions(
#     ${PROJECT_NAME}
#     PRIVATE "COMPOSITION_BUILDING_DLL"
# )
ament_target_dependencies( ${PROJECT_NAME} 
    rclcpp 
    geometry_msgs 
    sensor_msgs
    tf2_geometry_msgs
)
# link vn library
target_link_libraries( ${PROJECT_NAME} 
    vncxx
)
rclcpp_components_register_nodes(${PROJECT_NAME} "vectornav::VectornavNode")
set(node_plugs "${node_plugins}vectornav::VectornavNode;$<TARGET_FILE:${PROJECT_NAME}>\n")

# testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/test.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    geometry_msgs 
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} vncxx)
endif()


# install nodes
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# testing
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

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
