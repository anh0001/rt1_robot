# RT1 Description Package

This package contains the URDF description and visualization tools for the RT1 robot. It includes mesh files, URDF/XACRO descriptions, and launch files for visualizing the robot model.

## Package Structure

```
rt1_description/
├── CMakeLists.txt
├── launch/
│   ├── rt1_description.launch.py
│   └── rt1_rviz.launch.py
├── meshes/
│   ├── d435.dae
│   ├── hokuyo.dae
│   ├── rt1-caster.dae
│   ├── rt1-caswheel.dae
│   ├── rt1.dae
│   └── rt1-wheel.dae
├── package.xml
├── rviz/
│   └── rt1_visualization.rviz
├── urdf/
│   ├── caster.urdf.xacro
│   ├── d435.gazebo.xacro
│   ├── d435.urdf.xacro
│   ├── materials.urdf.xacro
│   ├── onebot_realrobot.urdf_ori.xacro
│   ├── onebot_realrobot.urdf.xacro
│   ├── onebot.urdf.xacro
│   └── wheel.urdf.xacro
└── world/
    └── w1.world
```

## Prerequisites

- ROS 2 Humble
- xacro package
- rviz2

## Installation

1. Clone this repository into your ROS 2 workspace's src directory:
```bash
cd ~/your_workspace/src
git clone <repository_url>
```

2. Build the package:
```bash
cd ~/your_workspace
colcon build --packages-select rt1_description
```

3. Source the workspace:
```bash
source ~/your_workspace/install/setup.bash
```

## Usage

### Checking XACRO Files

To validate and check the XACRO file syntax:
```bash
ros2 run xacro xacro src/rt1_description/urdf/onebot_realrobot.urdf.xacro
```

### Launching Visualization

#### Using Launch Files
To launch the robot visualization with joint state publisher GUI:
```bash
ros2 launch rt1_description rt1_description.launch.py gui:=true
```

#### Running RViz Directly
To open RViz with the predefined configuration:
```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix rt1_description)/share/rt1_description/rviz/rt1_visualization.rviz
```

## Package Components

### URDF Files
- `onebot_realrobot.urdf.xacro`: Main robot description file
- `caster.urdf.xacro`: Caster wheel description
- `wheel.urdf.xacro`: Main wheel description
- `materials.urdf.xacro`: Material definitions
- `d435.urdf.xacro`: RealSense D435 camera description

### Mesh Files
- `rt1.dae`: Main robot body mesh
- `rt1-wheel.dae`: Wheel mesh
- `rt1-caster.dae`: Caster wheel mesh
- `hokuyo.dae`: Lidar sensor mesh
- `d435.dae`: RealSense camera mesh

### Launch Files
- `rt1_description.launch.py`: Main launch file for robot visualization
- `rt1_rviz.launch.py`: Launch file for RViz configuration

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request