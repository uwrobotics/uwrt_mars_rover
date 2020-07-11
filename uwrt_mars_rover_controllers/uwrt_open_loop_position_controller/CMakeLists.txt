cmake_minimum_required(VERSION 3.0.2)
project(uwrt_open_loop_position_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS controller_interface forward_command_controller pluginlib uwrt_mars_rover_msgs)

catkin_package(
  CATKIN_DEPENDS
    uwrt_mars_rover_msgs
)

# Declare catkin project
catkin_package(
    CATKIN_DEPENDS controller_interface forward_command_controller
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME}
  src/joint_group_position_controller.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(FILES position_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
