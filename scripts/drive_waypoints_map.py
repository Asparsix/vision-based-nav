#!/usr/bin/env python3
"""Closed-loop odom waypoint driver for full-road RTAB coverage."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


# Odom frame: spawn ≈ (0,0) facing +x  (world spawn x=-20)
# Cover E–W main, N–S cross, spur, and return passes.
WAYPOINTS: list[tuple[float, float]] = [
    (8.0, 0.0),
    (18.0, 0.0),
    (28.0, 0.0),
    (38.0, 0.0),      # near B (world ~+18)
    (38.0, 2.0),
    (28.0, 0.0),
    (20.0, 0.0),      # intersection
    (20.0, 8.0),
    (20.0, 16.0),     # toward C
    (18.0, 16.0),
    (20.0, 8.0),
    (20.0, 0.0),
    (20.0, -8.0),
    (20.0, -16.0),    # south arm
    (22.0, -16.0),
    (20.0, -8.0),
    (20.0, 0.0),
    (12.0, 0.0),
    (8.0, 0.0),
    (8.0, 8.0),       # spur / residential
    (8.0, 14.0),
    (6.0, 14.0),
    (8.0, 8.0),
    (8.0, 0.0),
    (0.0, 0.0),       # back near A
    (0.0, 3.0),
    (8.0, 0.0),
    (20.0, 0.0),
    (30.0, 0.0),
    (20.0, 0.0),
    (20.0, 12.0),
    (20.0, 0.0),
    (20.0, -12.0),
    (20.0, 0.0),
    (10.0, 0.0),
]


class WaypointDriver(Node):
    def __init__(self) -> None:
        super().__init__('waypoint_map_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.get_logger().info(f'Waypoint coverage: {len(WAYPOINTS)} goals')

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # yaw from quaternion
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.x = p.x
        self.y = p.y
        self.yaw = math.atan2(siny, cosy)
        self.have_odom = True

    def _spin_until_odom(self, timeout: float = 10.0) -> None:
        t0 = time.time()
        while not self.have_odom and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.have_odom:
            raise RuntimeError('No /odom')

    def run(self) -> None:
        self._spin_until_odom()
        t_all = time.time()
        for i, (gx, gy) in enumerate(WAYPOINTS):
            self.get_logger().info(
                f'[{i + 1}/{len(WAYPOINTS)}] go ({gx:.1f},{gy:.1f}) '
                f'from ({self.x:.1f},{self.y:.1f}) yaw={self.yaw:.2f} '
                f'elapsed={(time.time() - t_all) / 60.0:.1f} min'
            )
            self._go_to(gx, gy, timeout=90.0)
        stop = Twist()
        for _ in range(20):
            self.pub.publish(stop)
            time.sleep(0.05)
        self.get_logger().info(
            f'Waypoint drive done in {(time.time() - t_all) / 60.0:.1f} min '
            f'at ({self.x:.1f},{self.y:.1f})'
        )

    def _go_to(self, gx: float, gy: float, timeout: float) -> None:
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            dx = gx - self.x
            dy = gy - self.y
            dist = math.hypot(dx, dy)
            if dist < 0.55:
                return
            desired = math.atan2(dy, dx)
            err = math.atan2(math.sin(desired - self.yaw), math.cos(desired - self.yaw))
            tw = Twist()
            # turn in place if heading bad
            if abs(err) > 0.55:
                tw.linear.x = 0.05
                tw.angular.z = 0.85 * math.copysign(1.0, err)
            else:
                tw.linear.x = min(0.42, 0.15 + 0.25 * dist)
                tw.angular.z = 1.2 * err
            self.pub.publish(tw)
            time.sleep(0.05)
        self.get_logger().warn(f'timeout approaching ({gx:.1f},{gy:.1f})')


def main() -> None:
    rclpy.init()
    node = WaypointDriver()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
