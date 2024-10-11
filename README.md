# RT1 Robot

## Project Overview

RT1 Robot is a ROS 2-based project for controlling and managing the RT1 robotic platform. This project provides a comprehensive software stack for robot navigation, hardware interfacing, and sensor integration.

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
└── package.xml
```

## Prerequisites

- ROS 2 Humble
- Ubuntu 22.04 LTS
- Git
- Colcon build tools

## Setup Instructions

1. Create a ROS 2 workspace:
   ```bash
   mkdir -p ~/rt1_ws/src
   cd ~/rt1_ws/src
   ```

2. Clone the repository:
   ```bash
   git clone https://github.com/your_username/rt1_robot.git
   cd rt1_robot
   ```

3. Initialize and update submodules:
   ```bash
   git submodule update --init --recursive
   ```

4. Install dependencies:
   ```bash
   cd ~/rt1_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

5. Build the workspace:
   ```bash
   colcon build
   ```

6. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

To launch the RT1 robot with basic functionality:

```bash
ros2 launch rt1_bringup rt1_bringup.launch.py
```

TO launch the RT1 robot with teleop manual control
```bash
ros2 launch rt1_bringup rt1_with_teleop.launch.py
```

For navigation:

```bash
ros2 launch rt1_navigation navigation.launch.py
```

## Docker

To use the Dockerized development environment:

1. Build the Docker image:
   ```bash
   docker build -t rt1_robot_dev ./docker
   ```

2. Run the Docker container:
   ```bash
   docker run -it --rm --net=host rt1_robot_dev
   ```

## Contributing

We welcome contributions to the RT1 Robot project. Please read our [CONTRIBUTING.md](CONTRIBUTING.md) file for guidelines on how to contribute.

## License

This project is licensed under the [MIT License](LICENSE).

## Support

For support, please open an issue in the GitHub repository or contact the maintainers directly.

## Acknowledgements

- RT.Works for the RT1 robotic platform
- The ROS 2 community for their invaluable resources and support