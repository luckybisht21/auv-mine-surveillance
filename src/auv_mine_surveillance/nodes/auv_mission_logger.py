#!/usr/bin/env python3
"""
AUV Mission Logger — detects mines by proximity to known positions.
Reads AUV pose from /auv/odom and compares to mine database.
"""
import os, csv, math, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# Known mine positions from minefield.world
MINES = [
    {"id": "mine_01", "x": 12.0,  "y":  3.5,  "z": -40.0, "type": "SPHERE"},
    {"id": "mine_02", "x": 18.5,  "y": -6.2,  "z": -42.0, "type": "SPHERE"},
    {"id": "mine_03", "x": 25.0,  "y":  8.0,  "z": -44.0, "type": "SPHERE"},
    {"id": "mine_04", "x": 30.0,  "y": -2.0,  "z": -45.0, "type": "CYLINDER"},
    {"id": "mine_05", "x": -5.0,  "y": 15.0,  "z": -41.0, "type": "SPHERE"},
    {"id": "mine_06", "x":  8.0,  "y": -12.0, "z": -49.0, "type": "SPHERE"},
]
DETECTION_RADIUS = 8.0   # metres — sonar range

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

class MissionLogger(Node):
    def __init__(self):
        super().__init__('auv_mission_logger')

        os.makedirs('/tmp/auv_mission_logs', exist_ok=True)
        from datetime import datetime
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = f'/tmp/auv_mission_logs/mission_log_{ts}.csv'

        self._csv_file = open(self._csv_path, 'w', newline='')
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow([
            'timestamp_utc','seq_id',
            'x_coord_m','y_coord_m','depth_m',
            'roll_deg','pitch_deg','yaw_deg',
            'vx_ms','vy_ms','vz_ms',
            'threat_type','confidence_score',
            'sonar_range_m','bearing_deg',
            'object_id','notes'
        ])
        self._csv_file.flush()

        self._seq = 0
        self._detected = set()   # mines already logged
        self._pos = (0.0, 0.0, -20.0)
        self._vel = (0.0, 0.0, 0.0)
        self._rpy = (0.0, 0.0, 0.0)

        self.create_subscription(Odometry, '/auv/odom',
                                 self._odom_cb, SENSOR_QOS)
        self._det_pub = self.create_publisher(String, '/auv/detected_objects', 10)
        self.create_timer(0.2, self._timer_cb)   # 5 Hz

        self.get_logger().info(f'Mission Logger ready | CSV: {self._csv_path}')
        self.get_logger().info(f'Detection radius: {DETECTION_RADIUS}m | {len(MINES)} mines in database')

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self._pos = (p.x, p.y, p.z)
        v = msg.twist.twist.linear
        self._vel = (v.x, v.y, v.z)
        # Convert quaternion to euler
        q = msg.pose.pose.orientation
        norm = math.sqrt(q.x**2+q.y**2+q.z**2+q.w**2)
        if norm > 0.1:
            x,y,z,w = q.x/norm,q.y/norm,q.z/norm,q.w/norm
            roll  = math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
            pitch = math.degrees(math.asin(max(-1.0,min(1.0,2*(w*y-z*x)))))
            yaw   = math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
            self._rpy = (roll, pitch, yaw)

    def _timer_cb(self):
        import time
        self._seq += 1
        ax, ay, az = self._pos
        roll, pitch, yaw = self._rpy
        vx, vy, vz = self._vel
        now = time.time()

        threat = 'SAFE'
        confidence = 1.0
        sonar_range = 0.0
        bearing = 0.0
        obj_id = ''
        notes = 'navigation_only'

        # Check proximity to each mine
        for mine in MINES:
            dx = ax - mine['x']
            dy = ay - mine['y']
            dz = az - mine['z']
            dist_3d = math.sqrt(dx*dx + dy*dy + dz*dz)
            dist_2d = math.sqrt(dx*dx + dy*dy)

            if dist_3d < DETECTION_RADIUS:
                threat = 'MINE'
                # Confidence based on distance (closer = more confident)
                confidence = max(0.70, min(0.99, 1.0 - dist_3d/DETECTION_RADIUS * 0.3))
                sonar_range = round(dist_3d, 2)
                bearing = round(math.degrees(math.atan2(-dy, -dx)) % 360, 1)
                obj_id = mine['id']
                notes = f'proximity_detection_{mine["type"].lower()}'

                if mine['id'] not in self._detected:
                    self._detected.add(mine['id'])
                    self.get_logger().info(
                        f'*** MINE DETECTED: {mine["id"]} | '
                        f'dist={dist_3d:.1f}m | confidence={confidence:.2f} ***'
                    )
                    det_msg = String()
                    det_msg.data = (f'{mine["id"]},x={mine["x"]},y={mine["y"]},'
                                   f'z={mine["z"]},conf={confidence:.2f}')
                    self._det_pub.publish(det_msg)
                break

        self._writer.writerow([
            f'{now:.6f}', self._seq,
            f'{ax:.4f}', f'{ay:.4f}', f'{az:.4f}',
            f'{roll:.3f}', f'{pitch:.3f}', f'{yaw:.3f}',
            f'{vx:.4f}', f'{vy:.4f}', f'{vz:.4f}',
            threat, f'{confidence:.4f}',
            f'{sonar_range}', f'{bearing}',
            obj_id, notes
        ])
        self._csv_file.flush()

    def destroy_node(self):
        self._csv_file.close()
        self.get_logger().info(
            f'Mission complete | {len(self._detected)}/{len(MINES)} mines found | '
            f'Log: {self._csv_path}'
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
