from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the package
    package_dir = get_package_share_directory('rt1_bringup')
    
    # Path to the MQTT params file
    mqtt_params_file = os.path.join(package_dir, 'config', 'mqtt_params.yaml')
    
    return LaunchDescription([
        # Declare a launch argument for the MQTT params file
        DeclareLaunchArgument(
            'mqtt_params_file',
            default_value=mqtt_params_file,
            description='Full path to the MQTT parameters file to use'
        ),
        
        # Launch the MQTT bridge node
        Node(
            package='mqtt_client',
            executable='mqtt_client',
            name='mqtt_client',
            parameters=[LaunchConfiguration('mqtt_params_file')],
            output='screen',
        ),
    ])