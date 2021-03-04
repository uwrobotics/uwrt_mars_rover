cmake_minimum_required(VERSION 3.0.2)
project(uwrt_mars_rover_spiral)

find_package(catkin REQUIRED COMPONENTS 
    ar_track_alvar_msgs
    geometry_msgs
    message_generation
    std_msgs
    uwrt_mars_rover_utils
    uwrt_mars_rover_spiral
    uwrt_mars_rover_drive_to_ar
    uwrt_mars_rover_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

add_service_files(
    FILES
    state_machine.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS
    ar_track_alvar_msgs
    geometry_msgs
    message_generation
    std_msgs
    uwrt_mars_rover_utils
    uwrt_mars_rover_spiral
    uwrt_mars_rover_drive_to_ar
    uwrt_mars_rover_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_server src/state_machine_node.cpp)

target_link_libraries(
    ${PROJECT_NAME}_server
    ${catkin_LIBRARIES}
)

add_dependencies(
    ${PROJECT_NAME}_server
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
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
