#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rt1_bringup',  # Replace with your package name
            executable='mqtt_publisher.py',
            name='mqtt_publisher',
            output='screen',
            parameters=[
                {'mqtt_broker_host': '10.0.1.137'},
                {'mqtt_broker_port': 1883},
                {'mqtt_topic': 'rt1/sensors'},
                {'ros_topic': '/rosrt_rt1'},
                {'mqtt_qos': 0},
                {'mqtt_username': ''},
                {'mqtt_password': ''},
                {'mqtt_keepalive': 60},
                {'mqtt_client_id': 'ros2_mqtt_publisher'},
            ],
        )
    ])
