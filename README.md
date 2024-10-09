# rt1_robot

rt1_robot/
│
├── src/
│   ├── rt1_bringup/
│   │   ├── launch/
│   │   │   └── rt1_bringup.launch.py
│   │   └── config/
│   │       └── rt1_params.yaml
│   │
│   ├── rt1_description/
│   │   ├── urdf/
│   │   │   └── rt1.urdf.xacro
│   │   ├── meshes/
│   │   └── rviz/
│   │       └── rt1_visualization.rviz
│   │
│   ├── rt1_hardware/
│   │   ├── include/
│   │   │   └── rt1_hardware/
│   │   │       └── motor_controller.hpp
│   │   └── src/
│   │       └── motor_controller.cpp
│   │
│   ├── rt1_navigation/
│   │   ├── launch/
│   │   │   └── navigation.launch.py
│   │   └── config/
│   │       ├── nav2_params.yaml
│   │       └── costmap_params.yaml
│   │
│   └── rt1_drivers/
│       ├── hokuyo_driver/
│       │   └── hokuyo_node.py
│       └── serial_driver/
│           └── serial_node.py
│
├── .github/
│   └── workflows/
│       └── ros_ci.yaml
│
├── docker/
│   └── Dockerfile
│
├── README.md
├── LICENSE
└── package.xml


## Submodule Management

After cloning the repository, initialize and update the submodule:

```bash
git submodule init
git submodule update