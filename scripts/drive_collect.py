#!/usr/bin/env python3
"""Drive the UGV around the enclosed courtyard while collecting camera views."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


# Spawn default: x=-7, y=0, yaw=0 (facing +X / east on EW road)
# Courtyard roads ~18m EW x 16m NS inside brick walls.
SEGMENTS = [
    # East along main road through plaza
    (0.45, 0.0, 10.0),
    (0.0, 0.9, 1.8),      # turn north
    (0.45, 0.0, 8.0),     # NS road north
    (0.0, 1.05, 3.0),     # look around near north houses
    (0.35, 0.0, 3.0),
    (0.0, -1.05, 3.2),    # face south
    (0.45, 0.0, 14.0),    # long south pass
    (0.0, 0.95, 1.8),     # turn east
    (0.4, 0.0, 6.0),
    (0.0, 0.95, 1.8),     # turn north again
    (0.4, 0.0, 6.0),
    (0.15, 0.7, 4.0),     # weave / scan trees
    (0.35, -0.35, 5.0),
    (0.0, 1.2, 4.0),      # in-place spin
    (0.4, 0.0, 5.0),
    (0.0, -0.95, 1.9),
    (0.45, 0.0, 8.0),     # west-ish return on EW
    (0.2, 0.55, 3.0),
    (0.4, 0.0, 6.0),
    (0.0, 1.1, 3.5),      # spin scan furniture / walls
    (0.35, 0.25, 6.0),
    (0.25, -0.5, 5.0),
    (0.4, 0.0, 7.0),
    (0.0, 0.85, 2.5),
    (0.4, 0.0, 8.0),
    (0.15, -0.9, 3.0),
    (0.4, 0.0, 6.0),
    (0.0, 1.0, 4.0),
    (0.35, 0.0, 5.0),
    (0.0, 0.0, 1.0),
]


class ScriptedDriver(Node):
    def __init__(self) -> None:
        super().__init__('scripted_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info(f'Courtyard drive: {len(SEGMENTS)} segments…')

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
