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

add_library(${PROJECT_NAME}
    SHARED
    src/uwrt_can.cpp
    #src/uwrt_params.cpp
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
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


install(TARGETS ${PROJECT_NAME} 
    uwrt_can_test_node
    EXPORT ${PROJECT_NAME}Config
    
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
)

export(TARGETS
    ${PROJECT_NAME}
    NAMESPACE uwrt_mars_rover_utilities::
    FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
)


install(
    FILES 
        include/uwrt_mars_rover_utilities/uwrt_can.h
    DESTINATION
        include/uwrt_mars_rover_utilities
    COMPONENT
        Devel
)
install(
    EXPORT
    ${PROJECT_NAME}Config
    DESTINATION "${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake"
    NAMESPACE uwrt_mars_rover_utilities::
)
ament_export_include_directories(
    lib/uwrt_mars_rover_hw_bridge/include
    include
)

ament_export_libraries(
    # uwrt-mars-rover-hw-bridge
)

ament_export_dependencies(
    rclcpp
)

ament_package()
