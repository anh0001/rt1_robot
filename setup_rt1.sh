#!/bin/bash
   # File: setup_rt1.sh

   # Setup script for RT1 Robot

   # Run Hokuyo setup script
   ./src/rt1_drivers/scripts/setup_hokuyo.sh

   # Build the workspace
   colcon build

   # Source the workspace
   source install/setup.bash

   echo "RT1 Robot setup complete. You can now launch the robot with:"
   echo "ros2 launch rt1_bringup rt1_slam_nav.launch.py"