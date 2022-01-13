cmake_minimum_required(VERSION 3.8)
project(uwrt_mars_rover_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


if(BUILD_TESTING)
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
endif()

ament_package()
