cmake_minimum_required(VERSION 3.8)
project(rt1_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosrt_rt1 REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${rosrt_rt1_INCLUDE_DIRS}
)

# Declare the rt1_hardware_interface executable
add_executable(rt1_hardware_interface src/rt1_hardware/src/rt1_hardware_interface.cpp)
ament_target_dependencies(rt1_hardware_interface
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  rosrt_rt1
)

# Install executables
install(TARGETS
  rt1_hardware_interface
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# Add the test executable
add_executable(test_serial_comm src/rt1_hardware/test/test_serial_comm.cpp)
ament_target_dependencies(test_serial_comm
  rclcpp
  geometry_msgs
  rosrt_rt1
)

# Install the test executable
install(TARGETS
  test_serial_comm
  DESTINATION lib/${PROJECT_NAME}
)