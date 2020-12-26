cmake_minimum_required(VERSION 3.10.2)
project(uwrt_utils)

# Load catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS
        include
        lib/uwrt_mars_rover_hw_bridge/include
  LIBRARIES
        ${PROJECT_NAME}
        uwrt-mars-rover-hw-bridge
  CATKIN_DEPENDS
        roscpp
        std_msgs
)

# Adds CMakeLists from hw-bridge
add_subdirectory(lib/uwrt_mars_rover_hw_bridge)

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare and link the C++
add_library(${PROJECT_NAME}
  src/uwrt_can.cpp
  src/${PROJECT_NAME}_can_test_node.cpp
)

# add executable and set dependencies
add_executable(${PROJECT_NAME}_can_test_node src/${PROJECT_NAME}_can_test_node.cpp)
set_target_properties(${PROJECT_NAME}_can_test_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(${PROJECT_NAME}_can_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# link dependencies for executable
target_link_libraries(${PROJECT_NAME}_can_test_node
  ${PROJECT_NAME} 
  ${catkin_LIBRARIES}
)

# Install executable
install(TARGETS ${PROJECT_NAME}_can_test_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install libraries
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
