from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # Get the path to the urg_node2 package
    urg_node2_dir = get_package_share_directory('urg_node2')

    return LaunchDescription([
        # Declare a launch argument for the params file
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the ROS2 parameters file to use'
        ),
        
        # Declare a launch argument for the mode
        DeclareLaunchArgument(
            'mode',
            default_value='sensor',
            description='Operation mode: "teleop" or "sensor"'
        ),
        
        Node(
            package='rosrt_rt1',
            executable='rosrt_rt1_node',
            name='rosrt_rt1',
            parameters=[
                LaunchConfiguration('params_file'),
                {'mode': LaunchConfiguration('mode')}
            ],
            remappings=remappings_tuple,
        ),
        
        # Include the Hokuyo URG launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                urg_node2_dir, 'launch', 'urg_node2.launch.py'
            )]),
            launch_arguments={
                'auto_start': 'true',
                'node_name': 'hokuyo_node',
                'scan_topic_name': 'scan'
            }.items()
        ),

        # Add static transform publisher for laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        )
    ])