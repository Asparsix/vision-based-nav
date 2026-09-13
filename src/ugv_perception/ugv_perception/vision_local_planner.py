#!/usr/bin/env python3
"""Reactive local driver: steer from /perception/local_costmap onto /cmd_vel.

Prefers road (cost 0), accepts grass (medium), rejects trees (lethal).
Uses heading hysteresis + cmd smoothing to avoid left/right oscillation.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class VisionLocalPlanner(Node):
    def __init__(self) -> None:
        super().__init__('ugv_vision_driver')

        self.declare_parameter('costmap_topic', '/perception/local_costmap')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('enabled', True)
        self.declare_parameter('v_max', 0.28)
        self.declare_parameter('v_min', 0.10)
        self.declare_parameter('w_max', 0.45)
        self.declare_parameter('lookahead_min', 0.7)
        self.declare_parameter('lookahead_max', 3.0)
        self.declare_parameter('num_angles', 17)
        self.declare_parameter('lethal_threshold', 90)
        self.declare_parameter('unknown_cost', 40)
        self.declare_parameter('stop_dist', 0.70)
        self.declare_parameter('control_hz', 8.0)
        self.declare_parameter('steer_gain', 0.7)
        self.declare_parameter('cmd_alpha', 0.25)  # low = smoother
        self.declare_parameter('yaw_hysteresis', 6.0)  # score margin to switch heading
        self.declare_parameter('yaw_deadband', 0.08)

        self.enabled = bool(self.get_parameter('enabled').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.v_min = float(self.get_parameter('v_min').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.lookahead_min = float(self.get_parameter('lookahead_min').value)
        self.lookahead_max = float(self.get_parameter('lookahead_max').value)
        self.num_angles = max(5, int(self.get_parameter('num_angles').value))
        self.lethal = int(self.get_parameter('lethal_threshold').value)
        self.unknown_cost = float(self.get_parameter('unknown_cost').value)
        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.steer_gain = float(self.get_parameter('steer_gain').value)
        self.cmd_alpha = float(self.get_parameter('cmd_alpha').value)
        self.yaw_hysteresis = float(self.get_parameter('yaw_hysteresis').value)
        self.yaw_deadband = float(self.get_parameter('yaw_deadband').value)

        self._grid = None
        self._info = None
        self._filt_v = 0.0
        self._filt_w = 0.0
        self._chosen_yaw = 0.0
        self._block_count = 0
        self._tick = 0

        cmd_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('costmap_topic').value),
            self._on_costmap,
            10,
        )
        hz = float(self.get_parameter('control_hz').value)
        self.create_timer(1.0 / max(hz, 1.0), self._on_timer)

        self.get_logger().info(
            f'Vision driver (smoothed): → {cmd_topic}  '
            f'v_max={self.v_max} w_max={self.w_max} alpha={self.cmd_alpha}'
        )

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self._info = msg.info
        self._grid = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width
        )

    def _sample_cost(self, x: float, y: float) -> float:
        info = self._info
        grid = self._grid
        if info is None or grid is None:
            return self.unknown_cost
        ix = int((x - info.origin.position.x) / info.resolution)
        iy = int((y - info.origin.position.y) / info.resolution)
        if ix < 0 or iy < 0 or ix >= info.width or iy >= info.height:
            return self.unknown_cost
        c = int(grid[iy, ix])
        if c < 0:
            return self.unknown_cost
        return float(c)

    def _score_heading(self, yaw: float) -> tuple[float, float]:
        """Return (score, min_clear). Lower score is better."""
        ds = 0.20
        dist = self.lookahead_min
        total = 0.0
        n = 0
        clear = self.lookahead_max
        while dist <= self.lookahead_max:
            x = dist * math.cos(yaw)
            y = dist * math.sin(yaw)
            c = self._sample_cost(x, y)
            if c >= self.lethal:
                clear = min(clear, dist)
                # Heavy near-field penalty, lighter far away
                total += 200.0 + 80.0 * max(0.0, 2.0 - dist)
            else:
                # Strongly prefer road (0) over grass (~45)
                total += c * c / 20.0
            n += 1
            dist += ds
        # Prefer staying near previously chosen heading (anti-chatter)
        total += 25.0 * abs(yaw - self._chosen_yaw)
        # Mild prefer straight
        total += 4.0 * abs(yaw)
        return total / max(n, 1), clear

    def _on_timer(self) -> None:
        self.enabled = bool(self.get_parameter('enabled').value)
        cmd = Twist()
        if not self.enabled or self._grid is None:
            self._filt_v *= 0.5
            self._filt_w *= 0.5
            cmd.linear.x = self._filt_v
            cmd.angular.z = self._filt_w
            self.pub.publish(cmd)
            return

        angles = np.linspace(-0.75, 0.75, self.num_angles)
        scores = []
        clears = []
        for yaw in angles:
            s, c = self._score_heading(float(yaw))
            scores.append(s)
            clears.append(c)

        best_i = int(np.argmin(scores))
        best_yaw = float(angles[best_i])
        best_score = float(scores[best_i])
        best_clear = float(clears[best_i])

        # Hysteresis: keep previous heading unless new is clearly better
        prev_i = int(np.argmin(np.abs(angles - self._chosen_yaw)))
        prev_score = float(scores[prev_i])
        if best_score + self.yaw_hysteresis < prev_score:
            self._chosen_yaw = best_yaw
        else:
            # slowly blend toward best
            self._chosen_yaw = 0.85 * self._chosen_yaw + 0.15 * best_yaw
            best_clear = float(clears[prev_i])
            best_score = prev_score

        # Debounce obstacle stops (need 3 consecutive frames)
        if best_clear <= self.stop_dist:
            self._block_count += 1
        else:
            self._block_count = max(0, self._block_count - 1)

        if self._block_count >= 3:
            target_v = 0.0
            target_w = self.w_max * 0.6 * (1.0 if self._chosen_yaw >= 0.0 else -1.0)
        else:
            speed_scale = max(0.0, min(1.0, 1.0 - best_score / 60.0))
            target_v = self.v_min + (self.v_max - self.v_min) * speed_scale
            yaw_cmd = self._chosen_yaw
            if abs(yaw_cmd) < self.yaw_deadband:
                yaw_cmd = 0.0
            target_w = max(-self.w_max, min(self.w_max, self.steer_gain * yaw_cmd))
            # Slow down while turning hard
            target_v *= max(0.35, 1.0 - 0.7 * abs(target_w) / max(self.w_max, 1e-3))

        a = max(0.05, min(1.0, self.cmd_alpha))
        self._filt_v = (1.0 - a) * self._filt_v + a * target_v
        self._filt_w = (1.0 - a) * self._filt_w + a * target_w

        cmd.linear.x = self._filt_v
        cmd.angular.z = self._filt_w
        self.pub.publish(cmd)

        self._tick += 1
        if self._tick == 1 or self._tick % 20 == 0:
            self.get_logger().info(
                f'drive v={cmd.linear.x:.2f} w={cmd.angular.z:.2f} '
                f'yaw*={self._chosen_yaw:.2f} score={best_score:.1f} clear={best_clear:.2f}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionLocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
