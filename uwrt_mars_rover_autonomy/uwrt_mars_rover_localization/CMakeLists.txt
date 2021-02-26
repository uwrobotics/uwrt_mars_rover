cmake_minimum_required(VERSION 3.0.2)
project(uwrt_mars_rover_localization)

find_package(catkin REQUIRED COMPONENTS 
    geometry_msgs
    nav_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    uwrt_mars_rover_utils
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    CATKIN_DEPENDS 
    geometry_msgs
    nav_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    uwrt_mars_rover_utils
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_odom src/publish_world_odom.cpp)

target_link_libraries(
    ${PROJECT_NAME}_odom
    ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_odom
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(TARGETS ${PROJECT_NAME}_odom
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )

install(DIRECTORY cfg launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
