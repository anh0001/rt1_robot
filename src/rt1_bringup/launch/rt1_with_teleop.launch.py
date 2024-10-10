from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Include the main RT1 bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rt1_bringup.launch.py'])
        ),
        
        # Launch the teleop node
        Node(
            package='rt1_bringup',
            executable='teleop_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix = 'xterm -e',
        ),
    ])