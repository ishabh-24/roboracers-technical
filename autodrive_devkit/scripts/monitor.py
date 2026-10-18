#!/usr/bin/env python3
"""Print lap count, lap time, best lap and collision count once a second.

Run inside the container after sourcing the workspace:
    python3 monitor.py
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from nav_msgs.msg import Odometry


class Monitor(Node):
    def __init__(self):
        super().__init__('lap_monitor')
        self.v = {}
        base = '/autodrive/roboracer_1/'
        for name, typ in [('lap_count', Int32), ('lap_time', Float32), ('last_lap_time', Float32),
                          ('best_lap_time', Float32), ('collision_count', Int32),
                          ('throttle', Float32), ('steering', Float32)]:
            self.create_subscription(typ, base + name, lambda m, n=name: self.v.__setitem__(n, m.data), 1)
        self.create_subscription(Odometry, base + 'odom', lambda m: self.v.__setitem__('speed', (m.twist.twist.linear.x**2 + m.twist.twist.linear.y**2) ** 0.5), 1)
        self.create_timer(1.0, self.tick)

    def tick(self):
        if not self.v:
            print('waiting for simulator data...', flush=True)
            return
        print(' '.join('%s=%s' % (k, round(v, 3) if isinstance(v, float) else v) for k, v in sorted(self.v.items())), flush=True)


def main():
    rclpy.init()
    rclpy.spin(Monitor())


if __name__ == '__main__':
    main()
