# CMake generated Testfile for 
# Source directory: /home/abdallah/Projects/ros2_ws/src/machathon5.00/car_demo
# Build directory: /home/abdallah/Projects/ros2_ws/src/machathon5.00/build/car_demo
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(PriusHybridPluginTest "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/abdallah/Projects/ros2_ws/src/machathon5.00/build/car_demo/test_results/car_demo/PriusHybridPluginTest.gtest.xml" "--package-name" "car_demo" "--output-file" "/home/abdallah/Projects/ros2_ws/src/machathon5.00/build/car_demo/ament_cmake_gtest/PriusHybridPluginTest.txt" "--command" "/home/abdallah/Projects/ros2_ws/src/machathon5.00/build/car_demo/PriusHybridPluginTest" "--gtest_output=xml:/home/abdallah/Projects/ros2_ws/src/machathon5.00/build/car_demo/test_results/car_demo/PriusHybridPluginTest.gtest.xml")
set_tests_properties(PriusHybridPluginTest PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/abdallah/Projects/ros2_ws/src/machathon5.00/build/car_demo/PriusHybridPluginTest" TIMEOUT "60" WORKING_DIRECTORY "/home/abdallah/Projects/ros2_ws/src/machathon5.00/build/car_demo" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/abdallah/Projects/ros2_ws/src/machathon5.00/car_demo/CMakeLists.txt;85;ament_add_gtest;/home/abdallah/Projects/ros2_ws/src/machathon5.00/car_demo/CMakeLists.txt;0;")
subdirs("gtest")
