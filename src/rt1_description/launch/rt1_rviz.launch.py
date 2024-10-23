# rt1_description/launch/rt1_rviz.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the package
    package_dir = get_package_share_directory('rt1_description')
    
    # Path to the RViz configuration file
    rviz_config = os.path.join(package_dir, 'rviz', 'rt1_visualization.rviz')
    
    # Create RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        rviz_node
    ])