cmake_minimum_required(VERSION 3.5)
project(car_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

find_package(prius_msgs REQUIRED)

find_package(gazebo 11 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(prius_description)

link_directories(${gazebo_dev_LIBRARY_DIRS})



add_library(PriusHybridPlugin SHARED 
  plugins/PriusHybridPlugin.cc
)
target_include_directories(PriusHybridPlugin PUBLIC include ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})
ament_target_dependencies(PriusHybridPlugin
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "prius_msgs"
)
ament_export_libraries(PriusHybridPlugin)

ament_export_include_directories(include)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(prius_msgs)

if(NOT WIN32)
  if(NOT APPLE)
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_plugins
      "prepend-non-duplicate;LD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  else()
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_plugins
      "prepend-non-duplicate;DYLD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  endif()
endif()
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_plugins.sh.in")

# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/prius_teleop_keyboard.py scripts/solution.py scripts/despawn_prius.py scripts/spawn_prius.py scripts/judge.py
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS PriusHybridPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(PriusHybridPluginTest plugins/PriusHybridPlugin.cc)  # Need to link .cpp file under test
  target_include_directories(PriusHybridPluginTest PUBLIC plugins/)  # Need to specify directories in order to #include from them in the test
  ament_target_dependencies(PriusHybridPluginTest gazebo_dev gazebo_ros rclcpp prius_msgs)  # Gtest requires rclcpp to instantiate the Node

endif()


ament_package()

install(
  DIRECTORY launch models worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    PriusHybridPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
