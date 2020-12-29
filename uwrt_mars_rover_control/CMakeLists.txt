cmake_minimum_required(VERSION 3.0.2)
project(uwrt_mars_rover_control)

find_package(catkin REQUIRED COMPONENTS
        combined_robot_hw
        controller_manager
        hardware_interface
        roscpp
        uwrt_mars_rover_drivetrain_hw
        )

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        combined_robot_hw
        controller_manager
        hardware_interface
        roscpp
        uwrt_mars_rover_drivetrain_hw
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}_loop.cpp
        src/${PROJECT_NAME}_loop_real.cpp
        )

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE
        )

install(DIRECTORY cfg launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
