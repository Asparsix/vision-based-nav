#!/usr/bin/env python3
"""Drive the UGV along a scripted path while publishing /cmd_vel (sim time)."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


# (linear_m_s, angular_rad_s, duration_s) — wall clock; fine for Gazebo real_time_factor~1
SEGMENTS = [
    # start near A facing east — go toward intersection
    (0.55, 0.0, 8.0),
    (0.35, 0.45, 2.0),   # slight weave
    (0.55, 0.0, 6.0),
    (0.2, 0.9, 1.8),     # turn left toward north road
    (0.5, 0.0, 7.0),
    (0.2, -1.0, 1.8),    # turn back south through intersection
    (0.55, 0.0, 10.0),
    (0.2, 0.95, 1.9),    # turn east toward B
    (0.55, 0.0, 10.0),
    (0.25, 0.7, 2.2),    # look at houses
    (0.45, -0.35, 4.0),
    (0.5, 0.0, 6.0),
    (0.15, 1.1, 2.5),    # spin scan
    (0.45, 0.0, 5.0),
    (0.2, -0.9, 2.0),
    (0.5, 0.0, 8.0),
    (0.0, 0.8, 3.0),     # in-place look around
    (0.4, 0.2, 5.0),
    (0.0, 0.0, 1.0),
]


class ScriptedDriver(Node):
    def __init__(self) -> None:
        super().__init__('scripted_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info(f'Driving {len(SEGMENTS)} segments…')

    def run(self) -> None:
        for i, (lin, ang, dur) in enumerate(SEGMENTS):
            tw = Twist()
            tw.linear.x = float(lin)
            tw.angular.z = float(ang)
            self.get_logger().info(
                f'[{i + 1}/{len(SEGMENTS)}] v={lin:.2f} w={ang:.2f} t={dur:.1f}s'
            )
            t0 = time.time()
            while time.time() - t0 < dur:
                self.pub.publish(tw)
                rclpy.spin_once(self, timeout_sec=0.05)
                time.sleep(0.05)
        stop = Twist()
        for _ in range(10):
            self.pub.publish(stop)
            time.sleep(0.05)
        self.get_logger().info('Drive finished.')


def main() -> None:
    rclpy.init()
    node = ScriptedDriver()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
