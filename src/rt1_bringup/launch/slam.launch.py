import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to packages
    rt1_bringup_dir = get_package_share_directory('rt1_bringup')
    slam_params_file = os.path.join(rt1_bringup_dir, 'config', 'slam_toolbox_params.yaml')

    # Include the robot bringup launch file
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rt1_bringup_dir, 'launch', 'rt1_bringup.launch.py')
        )
    )

    # Launch slam_toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',  # Use 'async_slam_toolbox_node' if preferred
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
    )

    return LaunchDescription([
        robot_bringup,
        slam_toolbox_node,
    ])
