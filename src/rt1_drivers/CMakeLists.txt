cmake_minimum_required(VERSION 3.5)
project(rt1_drivers)

# Find ament packages and other dependencies
find_package(ament_cmake REQUIRED)

find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# If you have C++ nodes or libraries, find those packages as well
# find_package(rclcpp REQUIRED)
# find_package(urg_node REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(
  PROGRAMS
    hokuyo_driver/hokuyo_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install configuration files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# If you have any C++ libraries or executables, uncomment and modify the following sections

# Add include directories
# include_directories(
#   include
# )

# Add executables
# add_executable(<executable_name> src/<source_file.cpp>)
# ament_target_dependencies(<executable_name> rclcpp ...)

# Install C++ executables
# install(
#   TARGETS <executable_name>
#   DESTINATION lib/${PROJECT_NAME}
# )

# ament_export_dependencies()

# Mark the package
ament_package()
