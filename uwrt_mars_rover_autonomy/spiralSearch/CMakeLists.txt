cmake_minimum_required(VERSION 3.0.2)
project(spiralSearch)

find_package(catkin REQUIRED COMPONENTS 
    actionlib_msgs
    actionlib
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
    DIRECTORY action
    FILES spiralSearch.action
)

generate_messages(
    DEPENDENCIES std_msgs actionlib_msgs geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS actionlib_msgs roscpp rospy std_msgs geometry_msgs message_runtime
)

include_directories(
# include
${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_server src/spiralSearch_server.cpp)
add_executable(${PROJECT_NAME}_client src/spiralSearch_client.cpp)

target_link_libraries(
    ${PROJECT_NAME}_server
    ${catkin_LIBRARIES}
)
target_link_libraries(
    ${PROJECT_NAME}_client
    ${catkin_LIBRARIES}
)
