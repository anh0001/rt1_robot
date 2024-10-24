#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

MSG = """
RT1 Robot Teleop Control
------------------------
Control the robot using:
   i : move forward
   k : move backward
   j : turn left
   l : turn right

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key: force stop
anything else : stop smoothly

CTRL-C to quit
"""

class TeleopKeyboard(Node):

    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # Declare and get the cmd_vel_topic parameter
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.x = 0.0
        self.th = 0.0
        
        self.get_logger().info(f'Publishing to {cmd_vel_topic}')

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def vels(self, linear_speed, angular_speed):
        return f"currently:\tlinear speed {linear_speed:.2f}\tangular speed {angular_speed:.2f}"

    def run(self):
        global settings
        try:
            print(MSG)
            print(self.vels(self.linear_speed, self.angular_speed))
            
            while True:
                key = self.getKey()
                if key == 'i':
                    self.x = self.linear_speed
                    self.th = 0.0
                elif key == 'k':
                    self.x = -self.linear_speed
                    self.th = 0.0
                elif key == 'j':
                    self.x = self.linear_speed * 0.5  # Add some forward motion when turning
                    self.th = self.angular_speed
                elif key == 'l':
                    self.x = self.linear_speed * 0.5  # Add some forward motion when turning
                    self.th = -self.angular_speed
                elif key == ' ':
                    self.x = 0.0
                    self.th = 0.0
                elif key == 'q':
                    self.linear_speed = min(self.linear_speed * 1.1, 1.0)
                    self.angular_speed = min(self.angular_speed * 1.1, 1.0)
                elif key == 'z':
                    self.linear_speed = max(self.linear_speed * 0.9, 0.1)
                    self.angular_speed = max(self.angular_speed * 0.9, 0.1)
                elif key == 'w':
                    self.linear_speed = min(self.linear_speed * 1.1, 1.0)
                elif key == 'x':
                    self.linear_speed = max(self.linear_speed * 0.9, 0.1)
                elif key == 'e':
                    self.angular_speed = min(self.angular_speed * 1.1, 1.0)
                elif key == 'c':
                    self.angular_speed = max(self.angular_speed * 0.9, 0.1)
                else:
                    self.x = 0.0
                    self.th = 0.0
                    if key == '\x03':
                        break

                twist = Twist()
                twist.linear.x = self.x
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = self.th
                self.publisher.publish(twist)

                print(self.vels(self.linear_speed, self.angular_speed))

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
            self.publisher.publish(twist)

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    
    teleop_keyboard = TeleopKeyboard()
    teleop_keyboard.run()

    teleop_keyboard.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()