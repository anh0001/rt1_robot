cmake_minimum_required(VERSION 3.8)
project(rt1_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../rosrt_rt1")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosrt_rt1 REQUIRED)
find_package(nav_msgs REQUIRED)  # Added for odometry messages
find_package(tf2 REQUIRED)       # Added for transforms
find_package(tf2_ros REQUIRED)   # Added for transform broadcaster

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${rosrt_rt1_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

# Declare the rt1_hardware_interface executable
add_executable(rt1_hardware_interface 
  src/rt1_hardware_interface.cpp 
  src/motor_controller.cpp
)
ament_target_dependencies(rt1_hardware_interface
  rclcpp
  geometry_msgs
  rosrt_rt1
)

# Declare the odometry calculator executable
add_executable(odometry_calculator_node
  src/odometry_calculator.cpp
)
ament_target_dependencies(odometry_calculator_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  rosrt_rt1
)

# Declare the test_serial_comm executable
add_executable(test_serial_comm 
  test/test_serial_comm.cpp
)
ament_target_dependencies(test_serial_comm
  rclcpp
  geometry_msgs
  rosrt_rt1
)

# Install executables
install(TARGETS
  rt1_hardware_interface
  odometry_calculator_node
  test_serial_comm
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

ament_package()