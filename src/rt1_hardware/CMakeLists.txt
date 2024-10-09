cmake_minimum_required(VERSION 3.8)
project(rt1_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosrt_rt1 REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${rosrt_rt1_INCLUDE_DIRS}
)

# Declare the rt1_hardware_interface executable
add_executable(rt1_hardware_interface src/rt1_hardware_interface.cpp src/motor_controller.cpp)
ament_target_dependencies(rt1_hardware_interface
  rclcpp
  geometry_msgs
  rosrt_rt1
)

# Install executables
install(TARGETS
  rt1_hardware_interface
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

ament_package()