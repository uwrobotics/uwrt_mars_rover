cmake_minimum_required(VERSION 3.0.2)
project(uwrt_mars_rover_spiral)

find_package(catkin REQUIRED COMPONENTS 
    actionlib    
    actionlib_msgs
    geometry_msgs
    message_generation
    std_msgs
    uwrt_mars_rover_utils
)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
    DIRECTORY action
    FILES spiralSearch.action
)

generate_messages(
    DEPENDENCIES actionlib_msgs geometry_msgs std_msgs
)

catkin_package(
    CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_runtime std_msgs uwrt_mars_rover_utils
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
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

add_dependencies(
    ${PROJECT_NAME}_server
    ${uwrt_mars_rover_spiral_EXPORTED_TARGETS}
)

add_dependencies(
    ${PROJECT_NAME}_client
    ${uwrt_mars_rover_spiral_EXPORTED_TARGETS}
)

install(TARGETS ${PROJECT_NAME}_server
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(TARGETS ${PROJECT_NAME}_server
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
