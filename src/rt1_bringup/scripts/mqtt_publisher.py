#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosrt_rt1.msg import Rt1Sensor  # Replace with your package and message name
import json
import paho.mqtt.client as mqtt
from rclpy.parameter import Parameter
from rosidl_runtime_py.convert import message_to_ordereddict

class MqttPublisherNode(Node):
    def __init__(self):
        super().__init__('mqtt_publisher')

        # Declare and read parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('mqtt_broker_host', 'localhost'),
                ('mqtt_broker_port', 1883),
                ('mqtt_topic', 'rt1/sensors'),
                ('ros_topic', 'rt1sensor_topic'),
                ('mqtt_qos', 0),
                ('mqtt_username', ''),
                ('mqtt_password', ''),
                ('mqtt_keepalive', 60),
                ('mqtt_client_id', 'ros2_mqtt_publisher'),
            ]
        )

        self.mqtt_broker_host = self.get_parameter('mqtt_broker_host').get_parameter_value().string_value
        self.mqtt_broker_port = self.get_parameter('mqtt_broker_port').get_parameter_value().integer_value
        self.mqtt_topic = self.get_parameter('mqtt_topic').get_parameter_value().string_value
        self.ros_topic = self.get_parameter('ros_topic').get_parameter_value().string_value
        self.mqtt_qos = self.get_parameter('mqtt_qos').get_parameter_value().integer_value
        self.mqtt_username = self.get_parameter('mqtt_username').get_parameter_value().string_value
        self.mqtt_password = self.get_parameter('mqtt_password').get_parameter_value().string_value
        self.mqtt_keepalive = self.get_parameter('mqtt_keepalive').get_parameter_value().integer_value
        self.mqtt_client_id = self.get_parameter('mqtt_client_id').get_parameter_value().string_value

        # Initialize MQTT client
        self.mqtt_client = mqtt.Client(client_id=self.mqtt_client_id)
        if self.mqtt_username and self.mqtt_password:
            self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
        self.mqtt_client.connect(self.mqtt_broker_host, self.mqtt_broker_port, self.mqtt_keepalive)

        # Create ROS subscriber
        self.subscription = self.create_subscription(
            Rt1Sensor,
            self.ros_topic,
            self.listener_callback,
            10  # QoS profile depth
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Convert ROS message to dictionary
        msg_dict = message_to_ordereddict(msg)
        # Convert dictionary to JSON string
        json_data = json.dumps(msg_dict)
        # Publish JSON string to MQTT
        self.mqtt_client.publish(self.mqtt_topic, json_data, qos=self.mqtt_qos)
        self.get_logger().info(f'Published to MQTT topic {self.mqtt_topic}: {json_data}')

def main(args=None):
    rclpy.init(args=args)
    node = MqttPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown the MQTT client gracefully
        node.mqtt_client.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
