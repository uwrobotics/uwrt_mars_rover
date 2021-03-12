cmake_minimum_required(VERSION 3.0.2)
project(uwrt_mars_rover_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(PROGRAMS
        scripts/bringup_can.sh
        scripts/bringup_fake_can.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
        )
