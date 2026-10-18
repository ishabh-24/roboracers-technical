#!/usr/bin/env python3
"""Record LiDAR, pose, speed and commands to an .npz for offline analysis.

    python3 recorder.py /path/out.npz
"""
import sys
import math
import signal
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32


class Recorder(Node):
    def __init__(self, out):
        super().__init__('recorder')
        self.out = out
        base = '/autodrive/roboracer_1/'
        self.create_subscription(LaserScan, base + 'lidar', self.on_scan, 1)
        self.create_subscription(Odometry, base + 'odom', self.on_odom, 1)
        self.create_subscription(Float32, base + 'throttle', lambda m: self.set('throttle', m.data), 1)
        self.create_subscription(Float32, base + 'steering', lambda m: self.set('steering', m.data), 1)
        self.create_subscription(Float32, base + 'throttle_command', lambda m: self.set('throttle_cmd', m.data), 1)
        self.create_subscription(Float32, base + 'steering_command', lambda m: self.set('steering_cmd', m.data), 1)
        self.create_subscription(Int32, base + 'collision_count', lambda m: self.set('collisions', m.data), 1)
        self.create_subscription(Int32, base + 'lap_count', lambda m: self.set('laps', m.data), 1)
        self.create_subscription(Float32, base + 'lap_time', lambda m: self.set('lap_time', m.data), 1)
        self.cur = {'throttle': 0.0, 'steering': 0.0, 'throttle_cmd': 0.0, 'steering_cmd': 0.0,
                    'collisions': 0, 'laps': 0, 'lap_time': 0.0}
        self.pose = None
        self.rows = []
        self.scans = []
        self.create_timer(5.0, self.save)

    def set(self, k, v):
        self.cur[k] = v

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        v = m.twist.twist.linear
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw, v.x, v.y, v.z, m.twist.twist.angular.z)

    def on_scan(self, m):
        if self.pose is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        c = self.cur
        self.rows.append([t, *self.pose, c['throttle'], c['steering'], c['throttle_cmd'], c['steering_cmd'],
                          c['collisions'], c['laps'], c['lap_time']])
        self.scans.append(np.asarray(m.ranges, dtype=np.float32))

    def save(self):
        if not self.rows:
            return
        np.savez_compressed(self.out, rows=np.array(self.rows), scans=np.array(self.scans),
                            columns='t x y yaw vx vy vz wz throttle steering throttle_cmd steering_cmd collisions laps lap_time')
        self.get_logger().info('saved %d samples' % len(self.rows))


def main():
    rclpy.init()
    node = Recorder(sys.argv[1] if len(sys.argv) > 1 else '/tmp/record.npz')
    signal.signal(signal.SIGTERM, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save()


if __name__ == '__main__':
    main()
