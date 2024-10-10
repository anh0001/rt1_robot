import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

MSG = """
RT1 Robot Teleop Control
------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

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
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed = 0.5
        self.turn = 1.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.status = 0.0

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)

    def run(self):
        try:
            print(MSG)
            print(self.vels(self.speed,self.turn))
            while True:
                key = self.get_key()
                if key == 'u':
                    self.x = self.speed
                    self.y = self.speed
                    self.z = 0.0
                    self.th = 0.0
                elif key == 'i':
                    self.x = self.speed
                    self.y = 0.0
                    self.z = 0.0
                    self.th = 0.0
                elif key == 'o':
                    self.x = self.speed
                    self.y = -self.speed
                    self.z = 0.0
                    self.th = 0.0
                elif key == 'j':
                    self.x = 0.0
                    self.y = self.speed
                    self.z = 0.0
                    self.th = 0.0
                elif key == 'k':
                    self.x = 0.0
                    self.y = 0.0
                    self.z = 0.0
                    self.th = 0.0
                elif key == 'l':
                    self.x = 0.0
                    self.y = -self.speed
                    self.z = 0.0
                    self.th = 0.0
                elif key == 'm':
                    self.x = -self.speed
                    self.y = self.speed
                    self.z = 0.0
                    self.th = 0.0
                elif key == ',':
                    self.x = -self.speed
                    self.y = 0.0
                    self.z = 0.0
                    self.th = 0.0
                elif key == '.':
                    self.x = -self.speed
                    self.y = -self.speed
                    self.z = 0.0
                    self.th = 0.0
                elif key == ' ':
                    self.x = 0.0
                    self.y = 0.0
                    self.z = 0.0
                    self.th = 0.0
                elif key == 'q':
                    self.speed = min(self.speed * 1.1, 1.0)
                elif key == 'z':
                    self.speed = max(self.speed * 0.9, 0.1)
                elif key == 'w':
                    self.speed = min(self.speed * 1.1, 1.0)
                elif key == 'x':
                    self.speed = max(self.speed * 0.9, 0.1)
                elif key == 'e':
                    self.turn = min(self.turn * 1.1, 1.0)
                elif key == 'c':
                    self.turn = max(self.turn * 0.9, 0.1)
                else:
                    self.x = 0.0
                    self.y = 0.0
                    self.z = 0.0
                    self.th = 0.0
                    if (key == '\x03'):
                        break

                twist = Twist()
                twist.linear.x = self.x
                twist.linear.y = self.y
                twist.linear.z = self.z
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = self.th
                self.publisher.publish(twist)

                print(self.vels(self.speed, self.turn))

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    teleop_keyboard = TeleopKeyboard()
    teleop_keyboard.run()
    teleop_keyboard.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()