cmake_minimum_required(VERSION 3.8)
project(uwrt_mars_rover_utils)

# set C/C++ standard
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(
  ${PROJECT_NAME}
  INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)


install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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

ament_package()
