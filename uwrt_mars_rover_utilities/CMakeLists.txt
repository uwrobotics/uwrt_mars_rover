cmake_minimum_required(VERSION 3.8)
project(uwrt_mars_rover_utilities)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif ()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_subdirectory(lib/uwrt_mars_rover_hw_bridge)

add_library(${PROJECT_NAME}
    src/uwrt_can.cpp
    #src/uwrt_params.cpp
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    rclcpp_components
)

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(
    include
    lib/uwrt_mars_rover_hw_bridge/include
)

ament_export_libraries(
    uwrt-mars-rover-hw-bridge
)

ament_export_dependencies(
    rclcpp
)

ament_package()
