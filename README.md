# RT1 Robot

## Project Overview

RT1 Robot is a ROS 2-based project for controlling and managing the RT1 robotic platform. This project provides a comprehensive software stack for robot navigation, hardware interfacing, and sensor integration, including support for the Hokuyo LiDAR.

## Repository Structure

```
rt1_robot/
├── src/
│   ├── rt1_bringup/          # Launch files and parameters
│   ├── rt1_description/      # URDF models and visualizations
│   ├── rt1_hardware/         # Hardware interface
│   ├── rt1_navigation/       # Navigation stack configuration
│   └── rt1_drivers/          # Custom drivers for sensors
├── .github/                  # CI/CD workflows
├── docker/                   # Dockerfile for containerized development
├── README.md
├── LICENSE
├── package.xml
└── setup_rt1.sh              # Main setup script
```

## Prerequisites

- ROS 2 Humble
- Ubuntu 22.04 LTS
- Git
- Colcon build tools

## Setup Instructions

1. Clone the repository:
   ```bash
   git clone https://github.com/anh0001/rt1_robot.git
   cd rt1_robot
   ```

2. Initialize and update submodules:
   ```bash
   git submodule update --init --recursive
   ```

3. Run the setup script:
   ```bash
   ./setup_rt1.sh
   ```
   This script will:
   - Set up the RT1 and Hokuyo LiDAR Serial Comms (add user to dialout group, create udev rule)
   - Build the workspace
   - Source the setup files

4. If you've been added to the dialout group, log out and log back in for the changes to take effect.

## Usage

To launch floxglove bridge:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Import the foxglove panels.

To launch the RT1 robot with basic functionality:

```bash
ros2 launch rt1_bringup rt1_bringup.launch.py
```

To launch the RT1 robot with SLAM and navigation:

```bash
ros2 launch rt1_bringup rt1_slam_nav.launch.py
```

To send sensors data through MQTT:
```bash
ros2 launch rt1_bringup rt1_mqtt_bridge.launch.py
```

To launch the RT1 robot with teleop manual control:
```bash
ros2 launch rt1_bringup rt1_teleop.launch.py
```

For navigation:

```bash
ros2 launch rt1_navigation navigation.launch.py
```

## Troubleshooting

- If you have issues with the Hokuyo LiDAR:
  1. Ensure your user is in the `dialout` group
  2. Check if the udev rule is set up correctly
  3. Verify that the LiDAR is connected and recognized at `/dev/hokuyo`

  You can re-run the RT1 Serial and Hokuyo setup script at any time:
  ```bash
  ./src/rt1_drivers/scripts/setup_rt1_serial.sh
  ./src/rt1_drivers/scripts/setup_hokuyo.sh
  ```

## Support

For support, please open an issue in the GitHub repository or contact the maintainers directly.

## Acknowledgements

- RT.Works for the RT1 robotic platform
- The ROS 2 community for their invaluable resources and support
- Hokuyo for the LiDAR sensor