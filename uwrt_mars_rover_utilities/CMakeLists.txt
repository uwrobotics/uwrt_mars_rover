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
find_package(uwrt_mars_rover_hw_bridge REQUIRED)
find_package(uwrt_msgs REQUIRED)



include_directories(include)

add_library(${PROJECT_NAME}
    src/uwrt_can.cpp
    #src/uwrt_params.cpp
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    rclcpp_components
    uwrt_mars_rover_hw_bridge
)

add_executable(uwrt_can_test_node src/uwrt_can_test_node.cpp)
ament_target_dependencies(uwrt_can_test_node
    rclcpp
    std_msgs
    rclcpp_components
    uwrt_mars_rover_hw_bridge
    uwrt_msgs
)

target_link_libraries(uwrt_can_test_node
    ${PROJECT_NAME}
)

install(TARGETS
    ${PROJECT_NAME}
    uwrt_can_test_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
        )

ament_export_include_directories(
    include
)

ament_export_dependencies(
    rclcpp
)

ament_package()
