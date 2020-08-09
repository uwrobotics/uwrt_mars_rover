cmake_minimum_required(VERSION 3.10.2)
project(uwrt_mars_rover_hw_bridge)
find_package(catkin REQUIRED)
catkin_package(
        INCLUDE_DIRS lib/uwrt_mars_rover_hw_bridge/include
        LIBRARIES uwrt-mars-rover-hw-bridge
)

add_subdirectory(lib/${PROJECT_NAME})
