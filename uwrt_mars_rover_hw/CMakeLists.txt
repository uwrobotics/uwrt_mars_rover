cmake_minimum_required(VERSION 3.0.2)
project(uwrt_mars_rover_hw)

find_package(catkin REQUIRED COMPONENTS
        hardware_interface
<<<<<<< HEAD
=======
        indexer_controller
        joint_limits_interface
>>>>>>> 4e2498a... initial science_hw base class implementation
        roscpp
        )

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        hardware_interface
<<<<<<< HEAD
=======
        indexer_controller
        joint_limits_interface
>>>>>>> 4e2498a... initial science_hw base class implementation
        roscpp
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

<<<<<<< HEAD
install(DIRECTORY include/${PROJECT_NAME}/
=======
## Declare a C++ library
add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}_control_loop.cpp
        src/${PROJECT_NAME}_control_loop_real.cpp
        src/${PROJECT_NAME}_drivetrain.cpp
        src/${PROJECT_NAME}_drivetrain_real.cpp
        src/${PROJECT_NAME}_science.cpp
        src/${PROJECT_NAME}_science_real.cpp
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
>>>>>>> 4e2498a... initial science_hw base class implementation
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE
        )

