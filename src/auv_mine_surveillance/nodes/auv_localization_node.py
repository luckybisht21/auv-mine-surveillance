#!/usr/bin/env python3
import math, threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, Imu, Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    TwistWithCovarianceStamped, PoseWithCovarianceStamped, TransformStamped, Quaternion
)
from tf2_ros import TransformBroadcaster

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

def safe_rotation_matrix(x, y, z, w):
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-6: return np.eye(3)
    x,y,z,w = x/norm,y/norm,z/norm,w/norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]
    ])

def safe_euler(x, y, z, w):
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if norm < 1e-6: return 0.0, 0.0, 0.0
    x,y,z,w = x/norm,y/norm,z/norm,w/norm
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1.0, min(1.0, 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


class DVLVelocityProcessor(Node):
    def __init__(self):
        super().__init__('dvl_processor')
        self._quat = [0.0, 0.0, 0.0, 1.0]
        self._beam_angle = math.radians(30.0)
        self.create_subscription(LaserScan, '/auv/dvl/ranges', self._dvl_cb, SENSOR_QOS)
        self.create_subscription(Imu, '/auv/imu/data', self._imu_cb, SENSOR_QOS)
        self._pub = self.create_publisher(TwistWithCovarianceStamped, '/auv/dvl/twist', 10)
        self.get_logger().info('DVL Processor ready.')

    def _imu_cb(self, msg):
        try:
            q = msg.orientation
            if math.sqrt(q.x**2+q.y**2+q.z**2+q.w**2) > 0.5:
                self._quat = [q.x, q.y, q.z, q.w]
        except Exception: pass

    def _dvl_cb(self, msg):
        try:
            ranges = list(msg.ranges)
            valid = [r for r in ranges if math.isfinite(r) and r > 0.1]
            if not valid: return
            sa = math.sin(self._beam_angle)
            while len(ranges) < 4:
                ranges.append(float(np.mean(valid)))
            vx = (ranges[0]-ranges[2])/(2.0*sa)*0.05
            vy = (ranges[1]-ranges[3])/(2.0*sa)*0.05
            if not (math.isfinite(vx) and math.isfinite(vy)):
                vx, vy = 0.0, 0.0
            R = safe_rotation_matrix(*self._quat)
            v = R @ np.array([vx, vy, 0.0])
            t = TwistWithCovarianceStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = 'odom'
            t.twist.twist.linear.x = float(v[0])
            t.twist.twist.linear.y = float(v[1])
            t.twist.twist.linear.z = float(v[2])
            cov = [0.0]*36
            cov[0]=0.01; cov[7]=0.01; cov[14]=0.02
            cov[21]=999.0; cov[28]=999.0; cov[35]=999.0
            t.twist.covariance = cov
            self._pub.publish(t)
        except Exception as e:
            self.get_logger().warn(f'DVL: {e}', throttle_duration_sec=5.0)


class AUVEKFLocalization(Node):
    def __init__(self):
        super().__init__('auv_ekf_localization')
        self._state = np.zeros(9)
        self._state[2] = -20.0
        self._P = np.eye(9) * 0.1
        self._Q = np.diag([0.001,0.001,0.001,0.001,0.001,0.002,0.01,0.01,0.01])
        self._last_time = self.get_clock().now()
        self._latest_imu = None
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Imu, '/auv/imu/data', self._imu_cb, SENSOR_QOS)
        self.create_subscription(TwistWithCovarianceStamped, '/auv/dvl/twist', self._dvl_cb, 10)
        self.create_subscription(Range, '/auv/depth/range', self._depth_cb, SENSOR_QOS)
        # KEY: listen to /set_pose to allow teleportation
        self.create_subscription(PoseWithCovarianceStamped, '/set_pose', self._set_pose_cb, 10)

        self._odom_pub = self.create_publisher(Odometry, '/auv/odom', 10)
        self._depth_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/auv/depth/pose', 10)
        self.create_timer(0.02, self._timer_cb)
        self.get_logger().info('AUV Localization ready — 50 Hz odom + TF.')

    def _imu_cb(self, msg):
        try:
            q = msg.orientation
            if math.sqrt(q.x**2+q.y**2+q.z**2+q.w**2) > 0.5:
                self._latest_imu = msg
        except Exception: pass

    def _dvl_cb(self, msg):
        try:
            lx = msg.twist.twist.linear.x
            ly = msg.twist.twist.linear.y
            lz = msg.twist.twist.linear.z
            if math.isfinite(lx): self._state[6] = lx
            if math.isfinite(ly): self._state[7] = ly
            if math.isfinite(lz): self._state[8] = lz
        except Exception: pass

    def _depth_cb(self, msg):
        try:
            if math.isfinite(msg.range) and msg.range > 0:
                self._state[2] = -abs(msg.range)
        except Exception: pass

    def _set_pose_cb(self, msg):
        """Teleport: instantly move AUV to requested position."""
        try:
            self._state[0] = msg.pose.pose.position.x
            self._state[1] = msg.pose.pose.position.y
            self._state[2] = msg.pose.pose.position.z
            self._state[6] = 0.0
            self._state[7] = 0.0
            self._state[8] = 0.0
            self.get_logger().info(
                f'TELEPORT → X={self._state[0]:.1f} '
                f'Y={self._state[1]:.1f} Z={self._state[2]:.1f}')
        except Exception as e:
            self.get_logger().warn(f'set_pose error: {e}')

    def _timer_cb(self):
        try:
            now = self.get_clock().now()
            dt = (now - self._last_time).nanoseconds / 1e9
            self._last_time = now
            if dt <= 0.0 or dt > 1.0: return

            self._state[0] += self._state[6] * dt
            self._state[1] += self._state[7] * dt
            self._state[2] += self._state[8] * dt

            if self._latest_imu is not None:
                q = self._latest_imu.orientation
                r,p,y = safe_euler(q.x, q.y, q.z, q.w)
                self._state[3]=r; self._state[4]=p; self._state[5]=y
            self._P += self._Q * dt

            orient = self._latest_imu.orientation if self._latest_imu else Quaternion(w=1.0)

            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = float(self._state[0])
            odom.pose.pose.position.y = float(self._state[1])
            odom.pose.pose.position.z = float(self._state[2])
            odom.pose.pose.orientation = orient
            odom.twist.twist.linear.x = float(self._state[6])
            odom.twist.twist.linear.y = float(self._state[7])
            odom.twist.twist.linear.z = float(self._state[8])
            cov = [0.0]*36
            for i,idx in enumerate([0,7,14,21,28,35]):
                cov[idx] = float(self._P[i,i])
            odom.pose.covariance = cov
            self._odom_pub.publish(odom)

            tf = TransformStamped()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x = float(self._state[0])
            tf.transform.translation.y = float(self._state[1])
            tf.transform.translation.z = float(self._state[2])
            tf.transform.rotation = orient
            self._tf_broadcaster.sendTransform(tf)
        except Exception as e:
            self.get_logger().warn(f'EKF: {e}', throttle_duration_sec=2.0)


def run(node):
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    try: ex.spin()
    except Exception: pass
    finally: ex.shutdown()


def main(args=None):
    rclpy.init(args=args)
    dvl = DVLVelocityProcessor()
    ekf = AUVEKFLocalization()
    t1 = threading.Thread(target=run, args=(dvl,), daemon=True)
    t2 = threading.Thread(target=run, args=(ekf,), daemon=True)
    t1.start(); t2.start()
    try: t1.join(); t2.join()
    except KeyboardInterrupt: pass
    finally:
        dvl.destroy_node(); ekf.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
