cmake_minimum_required(VERSION 2.8.3)
project(uwrt_mars_rover_neopixel)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        message_generation
        roscpp
        uwrt_mars_rover_msgs
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
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        uwrt_mars_rover_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}_can.cpp
        src/${PROJECT_NAME}_control.cpp
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
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

## Mark cpp header files for installation
install(DIRECTORY
        include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE
        )
