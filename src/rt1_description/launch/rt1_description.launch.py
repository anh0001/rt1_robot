from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    # GUI argument
    declared_arguments.append(
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Flag to enable joint_state_publisher_gui'
        )
    )
    
    # Use simulation argument
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Use simulation'
        )
    )
    
    # Serial port argument for real robot
    declared_arguments.append(
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for real robot'
        )
    )
    
    # Baud rate argument
    declared_arguments.append(
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for serial communication'
        )
    )

    # Get paths
    package_path = FindPackageShare('rt1_description')
    urdf_path = PathJoinSubstitution([package_path, 'urdf', 'onebot_realrobot.urdf.xacro'])
    
    # Robot description - with use_sim parameter
    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name='xacro'), ' ',
                urdf_path, ' ',
                'mesh_enabled:=true', ' ',
                'use_sim:=', LaunchConfiguration('use_sim')
            ]
        ),
        value_type=str
    )
    
    robot_description = {'robot_description': robot_description_content}
    
    # Controller configurations
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("rt1_description"), "config", "controllers.yaml"]
    )

    # Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    # Joint State Publisher (for simulation)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(LaunchConfiguration('use_sim')),
        parameters=[{
            'rate': 30
        }]
    )

    # Joint State Publisher GUI (optional, for testing)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Controller manager - with different configurations for sim vs real
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_file,
            {
                "use_sim": LaunchConfiguration('use_sim'),
                "serial_port": LaunchConfiguration('serial_port'),
                "baud_rate": LaunchConfiguration('baud_rate')
            }
        ],
        output="screen",
    )

    # Spawn controllers
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Nodes to launch
    nodes = [
        robot_state_pub_node,
        controller_manager_node,
        joint_broad_spawner,
        diff_drive_spawner,
        joint_state_publisher,
        joint_state_publisher_gui
    ]

    return LaunchDescription(declared_arguments + nodes)