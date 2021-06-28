cmake_minimum_required(VERSION 3.5)
project(uwrt_mars_rover_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  lib/uwrt_mars_rover_hw_bridge/include
  lib/uwrt_mars_rover_hw_bridge/can
  lib/uwrt_mars_rover_hw_bridge/can/generated
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_subdirectory(lib/uwrt_mars_rover_hw_bridge)

add_library(
  ${PROJECT_NAME}_library
  src/uwrt_can.cpp

)

ament_target_dependencies(
  ${PROJECT_NAME}_library
  "rclcpp"
)

ament_export_targets(
  ${PROJECT_NAME}_export 
  HAS_LIBRARY_TARGET
  )

ament_export_include_directories(
  include
  lib/uwrt_mars_rover_hw_bridge/include
)

ament_export_libraries(
  ${PROJECT_NAME}_library
  uwrt-mars-rover-hw-bridge
)

ament_export_dependencies(
  "rclcpp"
  )

# uwrt_mars_rover_hw_bridge
install(
  DIRECTORY lib/uwrt_mars_rover_hw_bridge/include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
install(
  TARGETS uwrt-mars-rover-hw-bridge
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# uwrt_mars_rover_utils
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}_library
  EXPORT ${PROJECT_NAME}_export
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


if (BUILD_TESTING)
  # generate compile_commands.json for clang-tidy
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

  # cppcheck
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck()

  # clang-format
  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format()

  # flake8
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()

  # xmllint
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

  # clang-tidy
  find_package(ament_cmake_clang_tidy REQUIRED)
  ament_clang_tidy(${CMAKE_BINARY_DIR})

  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)

endif ()

ament_package()
