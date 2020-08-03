cmake_minimum_required(VERSION 3.10.2)
project(uwrt_mars_rover_hw_bridge)
find_package(catkin REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES library/${PROJECT_NAME}/uwrt-mars-rover-hw-bridge
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_subdirectory(library/${PROJECT_NAME})

# adapted from uwrt_mars_rover_hw/CMakeLists.txt
############################################################################################
# Do i need this? This package cannot have an exe since it does not have a main right?
## Mark executables for installation
install(TARGETS library/${PROJECT_NAME}/uwrt-mars-rover-hw-bridge
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark libraries for installation
install(TARGETS
        uwrt-mars-rover-hw-bridge
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

## Mark cpp header files for installation
install(DIRECTORY
        library/${PROJECT_NAME}/include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE
        )