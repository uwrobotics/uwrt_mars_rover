cmake_minimum_required(VERSION 3.0.2)
project(uwrt_mars_rover_drive_to_ar)

find_package(catkin REQUIRED COMPONENTS 
    actionlib    
    actionlib_msgs
    geometry_msgs
    message_generation
    nav_msgs
    std_msgs
    tf
    uwrt_mars_rover_utils
)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
    DIRECTORY action
    FILES driveToAr.action
)

generate_messages(
    DEPENDENCIES actionlib_msgs geometry_msgs std_msgs
)

catkin_package(
    CATKIN_DEPENDS
    actionlib    
    actionlib_msgs
    geometry_msgs
    message_generation
    message_runtime
    nav_msgs
    std_msgs
    tf
    uwrt_mars_rover_utils
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_server src/drive_to_ar_server.cpp)

target_link_libraries(
    ${PROJECT_NAME}_server
    ${catkin_LIBRARIES}
)

add_dependencies(
    ${PROJECT_NAME}_server
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
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
