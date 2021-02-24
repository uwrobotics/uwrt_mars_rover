cmake_minimum_required(VERSION 2.8.3)
project(uwrt_mars_rover_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        actionlib_msgs
        sensor_msgs
        genmsg
        )

add_message_files(
        FILES
        NeopixelArrayMode.msg
        gps_heading.msg
)

add_service_files(
        FILES
        set_state.srv
)

add_action_files(
        FILES
        gps_goal.action
)

generate_messages(
        DEPENDENCIES
        std_msgs
        actionlib_msgs
        sensor_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
        actionlib_msgs
        sensor_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)
