cmake_minimum_required(VERSION 3.0.2)
project(uwrt_mars_rover_drivetrain_msgs)

find_package(
        catkin REQUIRED COMPONENTS
        geometry_msgs
        message_generation
        std_msgs
)

add_message_files(
        FILES
        OpenLoopTwist.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        geometry_msgs
        message_runtime
        std_msgs
)
