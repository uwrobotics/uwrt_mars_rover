cmake_minimum_required(VERSION 3.10.2)
project(uwrt_mars_rover_hw_bridge)
find_package(catkin REQUIRED)
catkin_package(
  INCLUDE_DIRS library/uwrt_mars_rover_hw_bridge/include
  LIBRARIES uwrt-mars-rover-hw-bridge
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_subdirectory(library/${PROJECT_NAME})

## Mark libraries for installation
install(TARGETS
        uwrt-mars-rover-hw-bridge
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )
