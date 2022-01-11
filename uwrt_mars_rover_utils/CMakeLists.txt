cmake_minimum_required(VERSION 3.8)
project(uwrt_mars_rover_utils)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)

add_library(uwrt_can src/uwrt_can.cpp)
target_compile_features(uwrt_can PUBLIC c_std_11 cxx_std_17)  # Require C11 and C++17
target_include_directories(uwrt_can PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
        uwrt_can
        rclcpp
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(uwrt_can PRIVATE "UWRT_MARS_ROVER_UTILS_BUILDING_LIBRARY")

install(
        DIRECTORY include
        DESTINATION include
)
install(
        TARGETS uwrt_can
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if (BUILD_TESTING)
    # generate compile_commands.json for clang-tidy
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "") # works

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
endif ()

ament_export_targets(
        ${PROJECT_NAME}Targets
)
ament_export_dependencies(
        rclcpp
)

ament_package()
