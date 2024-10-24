from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Get the bringup directory
    bringup_dir = get_package_share_directory('rt1_bringup')
    
    # Load the parameters file
    params_file = os.path.join(bringup_dir, 'config', 'rt1_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)
    
    # Extract remappings from the YAML file
    remappings = params['/**']['ros__parameters']['remappings']
    
    # Find the cmd_vel remapping
    cmd_vel_to = None
    for remap in remappings:
        if remap['from'] == '/cmd_vel':
            cmd_vel_to = remap['to']
            break
            
    if cmd_vel_to is None:
        cmd_vel_to = '/cmd_vel'  # Default if not found in remappings
        
    return LaunchDescription([
        # Include the main RT1 bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rt1_bringup.launch.py']),
            launch_arguments={'mode': 'teleop'}.items()  # Set mode to 'teleop'
        ),
        
        # Launch the teleop node
        Node(
            package='rt1_bringup',
            executable='teleop_keyboard.py',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -hold -e',
            # Use remapping from YAML
            parameters=[{
                'cmd_vel_topic': cmd_vel_to.lstrip('/')  # Remove leading slash for parameter
            }]
        ),
    ])