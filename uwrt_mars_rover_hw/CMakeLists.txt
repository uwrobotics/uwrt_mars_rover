cmake_minimum_required(VERSION 3.0.2)
project(uwrt_mars_rover_hw)

find_package(catkin REQUIRED COMPONENTS
        hardware_interface
        roscpp
        )

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        hardware_interface
        roscpp
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE
        )

