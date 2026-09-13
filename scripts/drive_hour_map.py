#!/usr/bin/env python3
"""~60 min closed-loop coverage drive for full-town RTAB mapping.

Stays on roads (odom waypoints), slow scans toward houses/trees.
NO Gazebo teleports (keeps odom continuous for a clean map).
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


# One "lap" of the town in odom frame (spawn ≈ world -20,0 → odom 0,0 facing +x)
# world x = odom_x - 20, world y = odom_y
LAP: list[tuple[float, float]] = [
    # A → intersection → B (east)
    (5.0, 0.0),
    (12.0, 0.0),
    (20.0, 0.0),
    (28.0, 0.0),
    (38.0, 0.0),
    # look north near B (houses)
    (38.0, 2.5),
    (36.0, 0.0),
    # B → A (west)
    (28.0, 0.0),
    (20.0, 0.0),
    (12.0, 0.0),
    (6.0, 0.0),
    (1.0, 0.0),
    # spur / residential (world x≈-12 → odom x≈8)
    (8.0, 0.0),
    (8.0, 6.0),
    (8.0, 12.0),
    (6.5, 14.0),
    (8.0, 10.0),
    (8.0, 2.0),
    (8.0, 0.0),
    # to intersection then north to C
    (20.0, 0.0),
    (20.0, 6.0),
    (20.0, 12.0),
    (20.0, 16.0),
    (18.0, 16.0),
    (20.0, 12.0),
    # south arm
    (20.0, 0.0),
    (20.0, -6.0),
    (20.0, -12.0),
    (20.0, -16.0),
    (22.0, -16.0),
    (20.0, -8.0),
    (20.0, 0.0),
    # west residential houses (south side glances)
    (14.0, -2.0),
    (10.0, 0.0),
    (6.0, 2.0),
    (2.0, 0.0),
    (0.0, 0.0),
]


class HourMapDriver(Node):
    def __init__(self, duration_min: float = 60.0) -> None:
        super().__init__('hour_map_driver')
        self.duration_s = duration_min * 60.0
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have = False
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 30)
        # fallback if alias only
        self.create_subscription(Odometry, '/odom', self._on_odom, 30)
        self.get_logger().info(f'Hour mapping drive target ≈ {duration_min:.0f} min')

    def _on_odom(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_of(msg.pose.pose.orientation)
        self.have = True

    def _wait_odom(self) -> None:
        t0 = time.time()
        while not self.have and time.time() - t0 < 30.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.have:
            raise RuntimeError('No /odometry/filtered or /odom')

    def _scan(self, seconds: float = 4.0) -> None:
        """In-place look left/right so houses enter the camera."""
        for w in (0.7, -0.7, 0.0):
            tw = Twist()
            tw.angular.z = w
            t0 = time.time()
            while time.time() - t0 < seconds / 3.0:
                self.pub.publish(tw)
                rclpy.spin_once(self, timeout_sec=0.05)
                time.sleep(0.05)

    def _go_to(self, gx: float, gy: float, timeout: float = 75.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            dx, dy = gx - self.x, gy - self.y
            dist = math.hypot(dx, dy)
            if dist < 0.6:
                return True
            desired = math.atan2(dy, dx)
            err = math.atan2(math.sin(desired - self.yaw), math.cos(desired - self.yaw))
            tw = Twist()
            if abs(err) > 0.6:
                tw.linear.x = 0.04
                tw.angular.z = 0.9 * math.copysign(1.0, err)
            else:
                tw.linear.x = min(0.38, 0.12 + 0.22 * dist)
                tw.angular.z = 1.15 * err
            self.pub.publish(tw)
            time.sleep(0.05)
        self.get_logger().warn(f'timeout ({gx:.1f},{gy:.1f}) at ({self.x:.1f},{self.y:.1f})')
        return False

    def run(self) -> None:
        self._wait_odom()
        t_all = time.time()
        lap = 0
        while time.time() - t_all < self.duration_s:
            lap += 1
            self.get_logger().info(
                f'=== LAP {lap}  elapsed={(time.time() - t_all) / 60.0:.1f} min  '
                f'pose=({self.x:.1f},{self.y:.1f}) ==='
            )
            for i, (gx, gy) in enumerate(LAP):
                if time.time() - t_all >= self.duration_s:
                    break
                self.get_logger().info(
                    f'[L{lap} {i + 1}/{len(LAP)}] → ({gx:.1f},{gy:.1f}) '
                    f'from ({self.x:.1f},{self.y:.1f})'
                )
                ok = self._go_to(gx, gy)
                # scan at key spots (near houses / ends)
                if i in (4, 5, 12, 15, 22, 28) or (ok and i % 7 == 0):
                    self._scan(5.0)
        stop = Twist()
        for _ in range(30):
            self.pub.publish(stop)
            time.sleep(0.05)
        self.get_logger().info(
            f'DONE after {(time.time() - t_all) / 60.0:.1f} min  '
            f'final=({self.x:.1f},{self.y:.1f}) laps≈{lap}'
        )


def main() -> None:
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--minutes', type=float, default=60.0)
    args = p.parse_args()
    rclpy.init()
    node = HourMapDriver(duration_min=args.minutes)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
