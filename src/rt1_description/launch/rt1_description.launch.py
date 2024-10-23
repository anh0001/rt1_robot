# rt1_description/launch/rt1_description.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the package
    package_dir = get_package_share_directory('rt1_description')
    
    # Path to the URDF file - using rt1.urdf.xacro as that's what we have
    urdf_file = os.path.join(package_dir, 'urdf', 'onebot_realrobot.urdf.xacro')
    
    # Declare the GUI argument
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Flag to enable joint_state_publisher_gui'
    )
    
    # Create robot description parameter
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' mesh_enabled:=true']),
        value_type=str
    )
    
    # Create a robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0,
        }]
    )
    
    # Create a joint_state_publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'rate': 30
        }]
    )
    
    # Optional: Add a joint_state_publisher_gui node for testing
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    return LaunchDescription([
        gui_arg,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui
    ])