#!/usr/bin/env python3
"""Long coverage drive for RTAB full-town mapping (~12–15 min).

World roads: E–W ~±24 m, N–S ~±20 m, spur near x=-12.
Publishes /cmd_vel; keep RTAB mapping running while this drives.
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# (linear_m_s, angular_rad_s, duration_s)
# Total duration ≈ sum of thirds ≈ 13–14 minutes at ~realtime.
SEGMENTS: list[tuple[float, float, float]] = [
    # --- Pass 1: A → intersection → B (eastbound) ---
    (0.45, 0.0, 12.0),
    (0.40, 0.15, 4.0),
    (0.45, 0.0, 14.0),
    (0.35, -0.12, 3.0),
    (0.45, 0.0, 18.0),
    (0.40, 0.10, 4.0),
    (0.45, 0.0, 16.0),
    (0.25, 0.0, 3.0),
    # look around near B
    (0.0, 0.70, 4.0),
    (0.0, -0.70, 4.0),
    # --- Turn around → westbound B → intersection → A ---
    (0.15, 1.05, 3.2),
    (0.45, 0.0, 18.0),
    (0.40, -0.10, 5.0),
    (0.45, 0.0, 18.0),
    (0.40, 0.12, 5.0),
    (0.45, 0.0, 16.0),
    (0.0, 0.65, 3.5),
    # --- North on cross road (toward C) ---
    (0.15, 0.95, 1.9),
    (0.42, 0.0, 14.0),
    (0.35, 0.20, 4.0),
    (0.42, 0.0, 12.0),
    (0.30, 0.0, 4.0),
    (0.0, 0.80, 3.5),
    (0.0, -0.80, 3.5),
    # --- South through intersection past south end ---
    (0.15, 1.05, 3.1),
    (0.42, 0.0, 18.0),
    (0.40, -0.15, 5.0),
    (0.42, 0.0, 16.0),
    (0.35, 0.12, 5.0),
    (0.42, 0.0, 14.0),
    (0.0, 0.70, 3.0),
    # --- Back north to intersection, then west residential ---
    (0.15, 1.05, 3.1),
    (0.42, 0.0, 20.0),
    (0.35, 0.0, 6.0),
    # turn west
    (0.15, 0.95, 1.9),
    (0.42, 0.0, 12.0),
    (0.35, 0.18, 5.0),
    (0.42, 0.0, 10.0),
    # --- Spur / residential loop near houses ---
    (0.15, 0.90, 2.0),
    (0.38, 0.0, 10.0),
    (0.30, 0.25, 5.0),
    (0.38, 0.0, 8.0),
    (0.15, 1.0, 2.2),
    (0.35, 0.0, 8.0),
    (0.0, 0.75, 4.0),
    (0.35, -0.20, 6.0),
    (0.38, 0.0, 10.0),
    # --- Return to main E–W, second full east pass ---
    (0.15, -0.95, 2.0),
    (0.42, 0.0, 10.0),
    (0.15, -0.95, 1.9),
    (0.45, 0.0, 20.0),
    (0.40, 0.15, 6.0),
    (0.45, 0.0, 18.0),
    (0.40, -0.12, 6.0),
    (0.45, 0.0, 16.0),
    # --- West again slowly (extra features) ---
    (0.15, 1.05, 3.2),
    (0.38, 0.0, 16.0),
    (0.32, 0.18, 6.0),
    (0.38, 0.0, 16.0),
    (0.32, -0.18, 6.0),
    (0.38, 0.0, 16.0),
    (0.32, 0.15, 5.0),
    (0.38, 0.0, 12.0),
    # --- Final north–south sweep ---
    (0.15, 0.95, 1.9),
    (0.40, 0.0, 14.0),
    (0.30, 0.22, 5.0),
    (0.40, 0.0, 12.0),
    (0.15, 1.05, 3.1),
    (0.40, 0.0, 20.0),
    (0.30, -0.18, 5.0),
    (0.40, 0.0, 16.0),
    (0.15, 1.05, 3.1),
    (0.40, 0.0, 12.0),
    # --- Finish near center, scan ---
    (0.25, 0.0, 6.0),
    (0.0, 0.60, 6.0),
    (0.0, -0.60, 6.0),
    (0.30, 0.0, 5.0),
    (0.0, 0.0, 2.0),
]


class FullMapDriver(Node):
    def __init__(self) -> None:
        super().__init__('full_map_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        total = sum(s[2] for s in SEGMENTS)
        self.get_logger().info(
            f'Full-area mapping drive: {len(SEGMENTS)} segments, ~{total / 60.0:.1f} min'
        )

    def run(self) -> None:
        t_all = time.time()
        for i, (lin, ang, dur) in enumerate(SEGMENTS):
            tw = Twist()
            tw.linear.x = float(lin)
            tw.angular.z = float(ang)
            elapsed = time.time() - t_all
            self.get_logger().info(
                f'[{i + 1}/{len(SEGMENTS)}] v={lin:.2f} w={ang:.2f} t={dur:.1f}s  '
                f'elapsed={elapsed / 60.0:.1f} min'
            )
            t0 = time.time()
            while time.time() - t0 < dur:
                self.pub.publish(tw)
                rclpy.spin_once(self, timeout_sec=0.05)
                time.sleep(0.05)
        stop = Twist()
        for _ in range(20):
            self.pub.publish(stop)
            time.sleep(0.05)
        self.get_logger().info(
            f'Drive finished in {(time.time() - t_all) / 60.0:.1f} min — leave RTAB running a few seconds to flush DB'
        )


def main() -> None:
    rclpy.init()
    node = FullMapDriver()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
