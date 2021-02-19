cmake_minimum_required(VERSION 2.8.3)
project(uwrt_mars_rover_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        )

add_message_files(
        FILES
        NeopixelArrayMode.msg
)

add_service_files(
        FILES
        set_state.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)
