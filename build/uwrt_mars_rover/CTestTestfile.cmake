# CMake generated Testfile for 
# Source directory: /home/uwrt/xbox_test_ws/src/uwrt_mars_rover/uwrt_mars_rover
# Build directory: /home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover/test_results/uwrt_mars_rover/xmllint.xunit.xml" "--package-name" "uwrt_mars_rover" "--output-file" "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover/ament_xmllint/xmllint.txt" "--command" "/opt/ros/galactic/bin/ament_xmllint" "--xunit-file" "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/build/uwrt_mars_rover/test_results/uwrt_mars_rover/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/uwrt_mars_rover" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/uwrt_mars_rover/CMakeLists.txt;9;ament_xmllint;/home/uwrt/xbox_test_ws/src/uwrt_mars_rover/uwrt_mars_rover/CMakeLists.txt;0;")
