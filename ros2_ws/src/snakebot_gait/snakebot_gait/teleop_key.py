"""Keyboard teleop for the snake over SSH: publishes geometry_msgs/Twist on /cmd_vel at 10 Hz.

    ros2 run snakebot_gait serpenoid --ros-args -p use_cmd_vel:=true   # terminal 1
    ros2 run snakebot_gait teleop_key                                  # terminal 2

Keys:  w / s   speed up / slow down (linear.x in steps of 0.25, -1..1; negative = backwards)
       a / d   steer left / right (angular.z in steps of 0.25, -1..1)
       x       straighten the steering
       space   stop (speed 0: the wave freezes in place)
       q       quit (publishes a stop first)

serpenoid scales these by its max_frequency and max_turn_bias parameters and freezes the wave if
/cmd_vel stops arriving, so killing this node stops the snake too.
"""
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

STEP = 0.25


def clamp(x):
    return max(-1.0, min(1.0, round(x, 2)))


class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 0.0
        self.turn = 0.0

    def publish(self):
        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = self.turn
        self.pub.publish(msg)

    def handle(self, key):
        if key == 'w':
            self.speed = clamp(self.speed + STEP)
        elif key == 's':
            self.speed = clamp(self.speed - STEP)
        elif key == 'a':
            self.turn = clamp(self.turn + STEP)
        elif key == 'd':
            self.turn = clamp(self.turn - STEP)
        elif key == 'x':
            self.turn = 0.0
        elif key == ' ':
            self.speed = 0.0
        else:
            return
        print(f'\rspeed {self.speed:+.2f}  turn {self.turn:+.2f}    ', end='', flush=True)


def main():
    if not sys.stdin.isatty():
        sys.exit('teleop_key needs an interactive terminal')
    rclpy.init()
    node = TeleopKey()
    old = termios.tcgetattr(sys.stdin)
    print(__doc__.split('\n\n')[2])
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready:
                key = sys.stdin.read(1).lower()
                if key == 'q':
                    break
                node.handle(key)
            node.publish()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.speed = node.turn = 0.0
        node.publish()
        print('\nstopped')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
