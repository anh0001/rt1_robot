from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    GroupAction,
    RegisterEventHandler
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('rt1_bringup')
    description_dir = get_package_share_directory('rt1_description')
    
    # Load the main parameters file
    params_file = os.path.join(bringup_dir, 'config', 'rt1_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)
    
    # Extract launch configurations from YAML
    launch_config = params['/**']['ros__parameters']['launch_config']
    
    # Declare launch arguments with defaults from YAML
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim',
            default_value=str(launch_config['use_sim']).lower(),
            description='Use simulation (true) or real hardware (false)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value=str(launch_config['use_rviz']).lower(),
            description='Start RViz2'
        ),
        DeclareLaunchArgument(
            'use_slam',
            default_value=str(launch_config['use_slam']).lower(),
            description='Start SLAM toolbox'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value=launch_config['mode'],
            description='Operation mode: "teleop" or "sensor"'
        )
    ]
    
    # Get path configurations
    slam_params_file = os.path.join(bringup_dir, 'config', 
                                   params['/**']['ros__parameters']['paths']['slam_params'])
    rviz_config = os.path.join(description_dir, 'rviz', 
                              params['/**']['ros__parameters']['paths']['rviz_config'])

    # Robot description launch
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(description_dir, 'launch', 'rt1_description.launch.py')
        ]),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'gui': 'false'
        }.items()
    )

    # Real hardware specific nodes
    real_hardware_group = GroupAction(
        actions=[
            # RT1 Robot Node
            Node(
                package='rosrt_rt1',
                executable='rosrt_rt1_node',
                name='rosrt_rt1',
                output='screen',
                parameters=[params_file],
                remappings=[(remap['from'], remap['to']) 
                           for remap in params['/**']['ros__parameters']['remappings']]
            ),
            
            # URG LiDAR Node
            Node(
                package='urg_node2',
                executable='urg_node2_node',
                name='urg_node2',
                parameters=[{
                    'serial_port': params['/**']['ros__parameters']['hardware']['lidar']['port'],
                    'serial_baud': params['/**']['ros__parameters']['hardware']['lidar']['baud_rate'],
                    'frame_id': params['/**']['ros__parameters']['hardware']['lidar']['frame_id'],
                    'angle_min': params['/**']['ros__parameters']['hardware']['lidar']['angle_min'],
                    'angle_max': params['/**']['ros__parameters']['hardware']['lidar']['angle_max'],
                    'scan_topic': params['/**']['ros__parameters']['hardware']['lidar']['topic']
                }]
            )
        ],
        condition=UnlessCondition(LaunchConfiguration('use_sim'))
    )

    # Simulation specific nodes
    sim_group = GroupAction(
        actions=[
            # # Gazebo simulation
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource([
            #         os.path.join(get_package_share_directory('gazebo_ros'), 
            #                    'launch', 'gazebo.launch.py')
            #     ])
            # ),
            
            # Spawn robot in Gazebo
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_entity',
                arguments=['-entity', 'rt1', '-topic', 'robot_description'],
                output='screen'
            )
        ],
        condition=IfCondition(LaunchConfiguration('use_sim'))
    )

    # Common nodes (used in both real and sim)
    common_group = GroupAction(
        actions=[
            # SLAM toolbox
            Node(
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_params_file],
                condition=IfCondition(LaunchConfiguration('use_slam'))
            ),
            
            # RViz
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
                condition=IfCondition(LaunchConfiguration('use_rviz'))
            )
        ]
    )
    
    # Controller spawner nodes
    controller_group = GroupAction(
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller'],
                output='screen',
            )
        ]
    )

    # Create and return launch description
    ld = LaunchDescription(declared_arguments)
    
    # Add all node groups
    ld.add_action(description_launch)
    ld.add_action(real_hardware_group)
    ld.add_action(sim_group)
    ld.add_action(common_group)
    ld.add_action(controller_group)
    
    return ld