cmake_minimum_required(VERSION 3.0.2)
project(diff_drive_voltage_controller)

set(CMAKE_CXX_STANDARD 17)

# Load catkin and all dependencies required for this package
find_package(
        catkin REQUIRED COMPONENTS
        control_msgs
        controller_interface
        forward_command_controller
        pluginlib
        uwrt_mars_rover_drivetrain_msgs
        uwrt_mars_rover_utils
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        control_msgs
        controller_interface
        forward_command_controller
        uwrt_mars_rover_drivetrain_msgs
        uwrt_mars_rover_utils
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(
        ${PROJECT_NAME}
        src/diff_drive_voltage_controller.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install library
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install headers
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)

# Install plugin xml
install(
        FILES ${PROJECT_NAME}_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
