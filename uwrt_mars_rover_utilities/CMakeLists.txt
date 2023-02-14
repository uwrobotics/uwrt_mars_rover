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

# include_directories(include)

add_library(${PROJECT_NAME} SHARED
    src/uwrt_can.cpp
    #src/uwrt_params.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/uwrt_mars_rover_utilities>
    $<INSTALL_INTERFACE:include/uwrt_mars_rover_utilities>
)

add_subdirectory(lib/uwrt_mars_rover_hw_bridge)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    rclcpp_components
)

add_executable(uwrt_can_test_node src/uwrt_can_test_node.cpp)
ament_target_dependencies(uwrt_can_test_node
    rclcpp
    std_msgs
    rclcpp_components
)

target_link_libraries(uwrt_can_test_node
    ${PROJECT_NAME}
)

ament_export_libraries(
    ${PROJECT_NAME}
    # uwrt-mars-rover-hw-bridge
)


ament_export_include_directories(
    include
    include/uwrt_mars_rover_utilities
    lib/uwrt_mars_rover_hw_bridge/include
)



ament_export_dependencies(
    rclcpp
)



install(
    DIRECTORY include/
    DESTINATION include
)

install(
    DIRECTORY include/uwrt_mars_rover_utilities/
    DESTINATION include
)


install(TARGETS ${PROJECT_NAME} 
    uwrt_can_test_node
    RUNTIME DESTINATION lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)


ament_package()
