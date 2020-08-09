cmake_minimum_required(VERSION 3.0.2)
project(uwrt_mars_rover_localization)

find_package(catkin REQUIRED roscpp std_msgs)
catkin_package(
    CATKIN_DEPENDS
    message_runtime
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(imu_pub src/imu_publisher.cpp)
target_link_libraries(imu_pub ${catkin_LIBRARIES})

add_executable(odom_pub src/odom_publisher.cpp)
target_link_libraries(odom_pub ${catkin_LIBRARIES})