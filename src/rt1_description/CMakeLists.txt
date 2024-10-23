cmake_minimum_required(VERSION 3.8)
project(rt1_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Install directories with resources
install(
  DIRECTORY meshes urdf rviz world launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()