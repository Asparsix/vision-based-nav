#!/usr/bin/env python3
"""Generic topic relay (Image / CameraInfo / OccupancyGrid / etc.)."""

from __future__ import annotations

import importlib

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def _resolve_type(type_str: str):
    # e.g. sensor_msgs/msg/Image
    pkg, _, rest = type_str.partition('/')
    kind, _, name = rest.partition('/')
    module = importlib.import_module(f'{pkg}.{kind}')
    return getattr(module, name)


class TopicRelay(Node):
    def __init__(self) -> None:
        super().__init__('topic_relay')
        self.declare_parameter('input_topic', '')
        self.declare_parameter('output_topic', '')
        self.declare_parameter('msg_type', 'sensor_msgs/msg/Image')
        inp = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value
        msg_type = _resolve_type(str(self.get_parameter('msg_type').value))
        self.pub = self.create_publisher(msg_type, out, qos_profile_sensor_data)
        self.create_subscription(
            msg_type, inp, lambda msg: self.pub.publish(msg), qos_profile_sensor_data
        )
        self.get_logger().info(f'Relay {inp} → {out}')


def main():
    rclpy.init()
    node = TopicRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
