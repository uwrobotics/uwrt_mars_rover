cmake_minimum_required(VERSION 3.0.2)
project(uwrt_mars_rover_drivetrain_hw)

find_package(catkin REQUIRED COMPONENTS
        hardware_interface
        joint_limits_interface
        roscpp
        transmission_interface
        uwrt_mars_rover_utils
        )

catkin_package(
        INCLUDE_DIRS
        include
        drivers/roboteq_driver/include
        LIBRARIES
        ${PROJECT_NAME}
        roboteq_driver
        CATKIN_DEPENDS
        hardware_interface
        joint_limits_interface
        roscpp
        transmission_interface
        uwrt_mars_rover_utils
)

include_directories(
        include
        drivers/roboteq_driver/include
        ${catkin_INCLUDE_DIRS}
)

add_library(roboteq_driver
        drivers/roboteq_driver/src/CanopenInterface.cpp
        drivers/roboteq_driver/src/RoboteqController.cpp
        )

add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}.cpp
        src/${PROJECT_NAME}_real.cpp
        )
target_link_libraries(${PROJECT_NAME} roboteq_driver ${catkin_LIBRARIES})

install(TARGETS
        roboteq_driver
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

install(DIRECTORY
        drivers/roboteq_driver/include/
        include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE
        )

install(DIRECTORY cfg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(FILES ${PROJECT_NAME}_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
