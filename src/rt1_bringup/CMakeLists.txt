cmake_minimum_required(VERSION 3.8)
project(rt1_bringup)

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
find_package(rt1_hardware REQUIRED)
find_package(rt1_description REQUIRED)
find_package(rt1_navigation REQUIRED)

# Declare a C++ executable
add_executable(rt1_bringup_node src/rt1_bringup_node.cpp)
ament_target_dependencies(rt1_bringup_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  rosrt_rt1
  rt1_hardware
  rt1_description
  rt1_navigation
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install executables
install(TARGETS
  rt1_bringup_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()