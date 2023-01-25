cmake_minimum_required(VERSION 3.10.2)
project(uwrt_test_stress_can)

#load catkin required dependencies
find_package(catkin REQUIRED COMPONENTS
        roscpp
    )



find_package(catkin REQUIRED COMPONENTS
    roscpp
)

catkin_package(
        INCLUDE_DIRS
        include
        lib/uwrt_mars_rover_hw_bridge/include
        LIBRARIES
        ${PROJECT_NAME}
        uwrt-mars-rover-hw-bridge
        CATKIN_DEPENDS
        roscpp
)


# Specify additional locations of header files
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# Declare and link the C++ Library
add_library(${PROJECT_NAME}
        src/uwrt_can.cpp
        src/uwrt_can_test_node.cpp
        )

install(TARGETS uwrt_test_stress_can
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )