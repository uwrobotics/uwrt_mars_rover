cmake_minimum_required(VERSION 2.8.3)
project(uwrt_arm_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  message_generation
  std_msgs
)

add_message_files(
  FILES
    UWRTArmCommand.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)

