cmake_minimum_required(VERSION 2.8.3)
project(uwrt_mars_rover_hw)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        combined_robot_hw
        controller_manager
        hardware_interface
        joint_limits_interface
        roscpp
        transmission_interface
        )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS
        include
        drivers/roboteq_driver/include
        LIBRARIES
        ${PROJECT_NAME}
        roboteq_driver
        CATKIN_DEPENDS
        combined_robot_hw
        controller_manager
        hardware_interface
        joint_limits_interface
        roscpp
        transmission_interface
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        drivers/roboteq_driver/include
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}_control_loop.cpp
        src/${PROJECT_NAME}_control_loop_real.cpp
        src/${PROJECT_NAME}_drivetrain.cpp
        src/${PROJECT_NAME}_drivetrain_real.cpp
        )

add_library(roboteq_driver
        drivers/roboteq_driver/src/CanopenInterface.cpp
        drivers/roboteq_driver/src/RoboteqController.cpp
        )

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}
        roboteq_driver
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
#install(PROGRAMS
#        scripts/my_python_script
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )

## Mark executables for installation
install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark libraries for installation
install(TARGETS
        roboteq_driver
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

## Mark cpp header files for installation
install(DIRECTORY
        drivers/roboteq_driver/include/
        include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE
        )

## Mark plugins for installation
install(FILES
        ${PROJECT_NAME}_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )