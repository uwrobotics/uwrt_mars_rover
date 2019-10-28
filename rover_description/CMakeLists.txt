cmake_minimum_required(VERSION 2.8.3)
project(rover_description)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY
    launch
    urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
