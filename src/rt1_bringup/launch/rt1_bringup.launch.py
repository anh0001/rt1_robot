from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosrt_rt1',
            executable='rosrt_rt1_node',
            name='rosrt_rt1',
            parameters=[{'port': '/dev/ttyUSB0'}],
            remappings=[('/cmd_vel', '/rt1/cmd_vel')]
        ),
        # Other nodes...
    ])