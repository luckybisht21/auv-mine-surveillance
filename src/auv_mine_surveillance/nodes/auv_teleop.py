#!/usr/bin/env python3
import sys, tty, termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Teleop(Node):
    def __init__(self):
        super().__init__('auv_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('TELEOP READY: W/S=surge  A/D=yaw  Q/E=heave  X=stop  Ctrl+C=quit')

    def key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def run(self):
        keys = {
            'w': (1, 0, 0, 0), 's': (-1, 0, 0, 0),
            'a': (0, 0, 0,  1), 'd': ( 0, 0, 0,-1),
            'q': (0, 0, 1, 0), 'e': ( 0, 0,-1, 0),
            'x': (0, 0, 0, 0),
        }
        while rclpy.ok():
            k = self.key()
            if k == '\x03': break
            if k in keys:
                lx, ly, lz, az = keys[k]
                t = Twist()
                t.linear.x=float(lx); t.linear.z=float(lz); t.angular.z=float(az)
                self.pub.publish(t)
                print(f'  Key={k}  surge={lx}  heave={lz}  yaw={az}    ', end='\r')

def main():
    rclpy.init()
    node = Teleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
