cmake_minimum_required(VERSION 3.8)
project(uwrt_mars_rover_drivetrain_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(uwrt_mars_rover_utilities REQUIRED)

add_library(
    ${PROJECT_NAME}
    SHARED
    src/${PROJECT_NAME}_actuator_interface.cpp
)
target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Require C99 and C++17
target_compile_features(
    ${PROJECT_NAME}
    PUBLIC
    c_std_11
    cxx_std_17
)
ament_target_dependencies(
    ${PROJECT_NAME}
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    transmission_interface
    uwrt_mars_rover_utilities
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "UWRT_MARS_ROVER_DRIVETRAIN_HW_BUILDING_LIBRARY")
pluginlib_export_plugin_description_file(hardware_interface ${PROJECT_NAME}.xml)

install(
    TARGETS ${PROJECT_NAME}
    DESTINATION lib
)

install(
    DIRECTORY
    include
    DESTINATION
    include
)

install(
    DIRECTORY
    config
    launch
    DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
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
endif()

ament_export_include_directories(
    include
)
ament_export_libraries(
    ${PROJECT_NAME}
)
ament_export_dependencies(
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    transmission_interface
)
ament_package()
