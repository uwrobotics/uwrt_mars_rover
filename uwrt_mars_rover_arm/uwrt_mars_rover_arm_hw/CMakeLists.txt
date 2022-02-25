cmake_minimum_required(VERSION 3.8)
project(uwrt_mars_rover_arm_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


add_custom_target(
        copy-compile-commands ALL
        ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_BINARY_DIR}/compile_commands.json
        ${CMAKE_CURRENT_LIST_DIR}
)




# set C/C++ standard
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# add_library(uwrt_mars_rover_utils SHARED IMPORTED)
target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
      uwrt_mars_rover_utils::uwrt_mars_rover_utils
)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(uwrt_mars_rover_utils REQUIRED)

include_directories(include)

# include header files
#target_include_directories(
#	${PROJECT_NAME}
#	PRIVATE
#	include
#)


# add dynamic library & it's dependecies
add_library(
	${PROJECT_NAME}
	SHARED
	src/${PROJECT_NAME}_actuator.cpp
    src/${PROJECT_NAME}_differential.cpp
)

ament_target_dependencies(
	${PROJECT_NAME}
	hardware_interface
	transmission_interface
	pluginlib
	rclcpp_lifecycle
	rclcpp
  uwrt_mars_rover_utils
)


target_compile_options(${PROJECT_NAME} PRIVATE -Wshadow -Werror) # add some extra flags for this target
target_compile_definitions(${PROJECT_NAME} PRIVATE "UWRT_MARS_ROVER_ARM_HW_DLL") # visibility header stuff regarding .so files
#TODO: fill in the above correct compile definition above
execute_process(
        COMMAND ${CMAKE_COMMAND} -E create_symlink
                ${CMAKE_BINARY_DIR}/compile_commands.json
                ${CMAKE_SOURCE_DIR}/compile_commands.json
        )


# install binaries in correct locations
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# xml file that is used by ros control to help load dynamic libraries
pluginlib_export_plugin_description_file(hardware_interface ${PROJECT_NAME}.xml)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # need json compile commands to help with lsp

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
