#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class ThrusterController(Node):
    def __init__(self):
        super().__init__('thruster_controller')
        self._port  = self.create_publisher(Float64, '/auv/thruster/port_surge/cmd_force', 10)
        self._stbd  = self.create_publisher(Float64, '/auv/thruster/stbd_surge/cmd_force', 10)
        self._heave = self.create_publisher(Float64, '/auv/thruster/heave/cmd_force', 10)
        self._yaw   = self.create_publisher(Float64, '/auv/thruster/yaw/cmd_force', 10)
        self.create_subscription(Twist, '/cmd_vel', self._cb, 10)
        self.get_logger().info('Thruster Controller ready')

    def _cb(self, msg):
        MAX = 80.0
        f = Float64()
        f.data = msg.linear.x * MAX
        self._port.publish(f); self._stbd.publish(f)
        f.data = msg.linear.z * MAX
        self._heave.publish(f)
        f.data = msg.angular.z * MAX
        self._yaw.publish(f)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ThrusterController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
