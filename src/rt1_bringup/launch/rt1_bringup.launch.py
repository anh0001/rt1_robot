from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Get the path to the package
    package_dir = get_package_share_directory('rt1_bringup')
    
    # Path to the params file
    params_file = os.path.join(package_dir, 'config', 'rt1_params.yaml')
    
    # Load the YAML file
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['rosrt_rt1']
    
    # Extract remappings from the YAML file
    remappings = params.get('remappings', [])
    remappings_tuple = [(remap['from'], remap['to']) for remap in remappings]
    
    return LaunchDescription([
        # Declare a launch argument for the params file
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the ROS2 parameters file to use'
        ),
        
        Node(
            package='rosrt_rt1',
            executable='rosrt_rt1_node',
            name='rosrt_rt1',
            parameters=[LaunchConfiguration('params_file')],
            remappings=remappings_tuple
        ),
        # Other nodes...
    ])