# Install script for directory: /home/uwrt/xbox_test_ws/src/uwrt_mars_rover/uwrt_mars_rover_utils/uwrt_mars_rover_xbox_controller

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/install/uwrt_mars_rover_xbox_controller")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/rosidl_interfaces" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_index/share/ament_index/resource_index/rosidl_interfaces/uwrt_mars_rover_xbox_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/uwrt_mars_rover_xbox_controller" TYPE DIRECTORY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/rosidl_generator_c/uwrt_mars_rover_xbox_controller/" REGEX "/[^/]*\\.h$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/environment" TYPE FILE FILES "/opt/ros/galactic/lib/python3.8/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/environment" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_environment_hooks/library_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_generator_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_generator_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/libuwrt_mars_rover_xbox_controller__rosidl_generator_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_generator_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_generator_c.so"
         OLD_RPATH "/opt/ros/galactic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_generator_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/uwrt_mars_rover_xbox_controller" TYPE DIRECTORY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/rosidl_typesupport_fastrtps_c/uwrt_mars_rover_xbox_controller/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_fastrtps_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_fastrtps_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_fastrtps_c.so"
         OLD_RPATH "/opt/ros/galactic/lib:/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_fastrtps_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/uwrt_mars_rover_xbox_controller" TYPE DIRECTORY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/rosidl_typesupport_fastrtps_cpp/uwrt_mars_rover_xbox_controller/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_fastrtps_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_fastrtps_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_fastrtps_cpp.so"
         OLD_RPATH "/opt/ros/galactic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_fastrtps_cpp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/uwrt_mars_rover_xbox_controller" TYPE DIRECTORY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/rosidl_typesupport_introspection_c/uwrt_mars_rover_xbox_controller/" REGEX "/[^/]*\\.h$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_c.so"
         OLD_RPATH "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller:/opt/ros/galactic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_c.so"
         OLD_RPATH "/opt/ros/galactic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/uwrt_mars_rover_xbox_controller" TYPE DIRECTORY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/rosidl_generator_cpp/uwrt_mars_rover_xbox_controller/" REGEX "/[^/]*\\.hpp$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/uwrt_mars_rover_xbox_controller" TYPE DIRECTORY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/rosidl_typesupport_introspection_cpp/uwrt_mars_rover_xbox_controller/" REGEX "/[^/]*\\.hpp$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cpp.so"
         OLD_RPATH "/opt/ros/galactic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cpp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_cpp.so"
         OLD_RPATH "/opt/ros/galactic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__rosidl_typesupport_cpp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/environment" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/environment" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/rosidl_generator_py/uwrt_mars_rover_xbox_controller/__init__.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/install/uwrt_mars_rover_xbox_controller/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/__init__.py"
      )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/msg" TYPE DIRECTORY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/rosidl_generator_py/uwrt_mars_rover_xbox_controller/msg/" REGEX "/[^/]*\\.py$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_fastrtps_c.cpython-38-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_fastrtps_c.cpython-38-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_fastrtps_c.cpython-38-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller" TYPE SHARED_LIBRARY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/rosidl_generator_py/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_fastrtps_c.cpython-38-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_fastrtps_c.cpython-38-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_fastrtps_c.cpython-38-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_fastrtps_c.cpython-38-x86_64-linux-gnu.so"
         OLD_RPATH "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/rosidl_generator_py/uwrt_mars_rover_xbox_controller:/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller:/opt/ros/galactic/lib:/opt/ros/galactic/share/std_msgs/cmake/../../../lib:/opt/ros/galactic/share/builtin_interfaces/cmake/../../../lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_fastrtps_c.cpython-38-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_introspection_c.cpython-38-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_introspection_c.cpython-38-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_introspection_c.cpython-38-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller" TYPE SHARED_LIBRARY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/rosidl_generator_py/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_introspection_c.cpython-38-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_introspection_c.cpython-38-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_introspection_c.cpython-38-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_introspection_c.cpython-38-x86_64-linux-gnu.so"
         OLD_RPATH "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/rosidl_generator_py/uwrt_mars_rover_xbox_controller:/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller:/opt/ros/galactic/lib:/opt/ros/galactic/share/std_msgs/cmake/../../../lib:/opt/ros/galactic/share/builtin_interfaces/cmake/../../../lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_introspection_c.cpython-38-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_c.cpython-38-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_c.cpython-38-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_c.cpython-38-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller" TYPE SHARED_LIBRARY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/rosidl_generator_py/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_c.cpython-38-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_c.cpython-38-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_c.cpython-38-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_c.cpython-38-x86_64-linux-gnu.so"
         OLD_RPATH "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/rosidl_generator_py/uwrt_mars_rover_xbox_controller:/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller:/opt/ros/galactic/lib:/opt/ros/galactic/share/std_msgs/cmake/../../../lib:/opt/ros/galactic/share/builtin_interfaces/cmake/../../../lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_c.cpython-38-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__python.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__python.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__python.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/rosidl_generator_py/uwrt_mars_rover_xbox_controller/libuwrt_mars_rover_xbox_controller__python.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__python.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__python.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__python.so"
         OLD_RPATH "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller:/opt/ros/galactic/share/std_msgs/cmake/../../../lib:/opt/ros/galactic/share/builtin_interfaces/cmake/../../../lib:/opt/ros/galactic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libuwrt_mars_rover_xbox_controller__python.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/msg" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/rosidl_adapter/uwrt_mars_rover_xbox_controller/msg/XboxController.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/msg" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/uwrt_mars_rover_utils/uwrt_mars_rover_xbox_controller/msg/XboxController.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller" TYPE DIRECTORY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/uwrt_mars_rover_utils/uwrt_mars_rover_xbox_controller/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libxbox_controller.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libxbox_controller.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libxbox_controller.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/libxbox_controller.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libxbox_controller.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libxbox_controller.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libxbox_controller.so"
         OLD_RPATH "/opt/ros/galactic/lib:/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller:/opt/ros/galactic/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libxbox_controller.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcoordinateNode.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcoordinateNode.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcoordinateNode.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/libcoordinateNode.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcoordinateNode.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcoordinateNode.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcoordinateNode.so"
         OLD_RPATH "/opt/ros/galactic/lib:/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller:/opt/ros/galactic/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcoordinateNode.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/uwrt_mars_rover_xbox_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/uwrt_mars_rover_xbox_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/environment" TYPE FILE FILES "/opt/ros/galactic/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/environment" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/environment" TYPE FILE FILES "/opt/ros/galactic/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/environment" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_index/share/ament_index/resource_index/packages/uwrt_mars_rover_xbox_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_generator_cExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_generator_cExport.cmake"
         "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/CMakeFiles/Export/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_generator_cExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_generator_cExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_generator_cExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/CMakeFiles/Export/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_generator_cExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/CMakeFiles/Export/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_generator_cExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cExport.cmake"
         "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/CMakeFiles/Export/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/CMakeFiles/Export/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/CMakeFiles/Export/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_cExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_cExport.cmake"
         "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/CMakeFiles/Export/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_cExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_cExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_cExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/CMakeFiles/Export/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_cExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/CMakeFiles/Export/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_cExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_generator_cppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_generator_cppExport.cmake"
         "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/CMakeFiles/Export/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_generator_cppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_generator_cppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_generator_cppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/CMakeFiles/Export/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_generator_cppExport.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cppExport.cmake"
         "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/CMakeFiles/Export/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/CMakeFiles/Export/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cppExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/CMakeFiles/Export/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_introspection_cppExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_cppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_cppExport.cmake"
         "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/CMakeFiles/Export/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_cppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_cppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_cppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/CMakeFiles/Export/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_cppExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/CMakeFiles/Export/share/uwrt_mars_rover_xbox_controller/cmake/uwrt_mars_rover_xbox_controller__rosidl_typesupport_cppExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/rclcpp_components" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_index/share/ament_index/resource_index/rclcpp_components/uwrt_mars_rover_xbox_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/rosidl_cmake/rosidl_cmake-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/rosidl_cmake/rosidl_cmake_export_typesupport_libraries-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/rosidl_cmake/rosidl_cmake_export_typesupport_targets-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller/cmake" TYPE FILE FILES
    "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_core/uwrt_mars_rover_xbox_controllerConfig.cmake"
    "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/ament_cmake_core/uwrt_mars_rover_xbox_controllerConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uwrt_mars_rover_xbox_controller" TYPE FILE FILES "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/uwrt_mars_rover_utils/uwrt_mars_rover_xbox_controller/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller__py/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover_xbox_controller/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
