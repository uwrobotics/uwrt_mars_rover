cmake_minimum_required(VERSION 3.10.2)
project(uwrt_mars_rover_utils)

# Load catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
        roscpp
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
)

# Adds CMakeLists from hw-bridge
add_subdirectory(lib/uwrt_mars_rover_hw_bridge)

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

# uwrt_can_test_node
add_executable(uwrt_can_test_node src/uwrt_can_test_node.cpp)
add_dependencies(uwrt_can_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uwrt_can_test_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

# uwrt_params_test_node
add_executable(uwrt_params_test_node src/uwrt_params_test_node.cpp)
add_dependencies(uwrt_params_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uwrt_params_test_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

# Install executables
install(TARGETS uwrt_can_test_node uwrt_params_test_node
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
