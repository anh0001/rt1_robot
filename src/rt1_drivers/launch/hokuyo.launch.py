from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urg_node_driver = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'frame_id': 'laser',
            'publish_intensity': False,
        }],
        output='screen'
    )

    return LaunchDescription([
        urg_node_driver
    ])
