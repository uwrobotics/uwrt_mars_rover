cmake_minimum_required(VERSION 3.5)
project(uwrt_mars_rover_drivetrain_description)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY
      config
      launch
      rviz
      urdf
    DESTINATION
      share/${PROJECT_NAME}
)

if (BUILD_TESTING)
  # flake8
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(
      CONFIG_FILE ${CMAKE_SOURCE_DIR}/../../.flake8
  )

  # xmllint
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif ()

ament_package()
