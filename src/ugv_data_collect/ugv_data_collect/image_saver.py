#!/usr/bin/env python3
"""Subscribe to a camera topic and save frames to disk for labeling/training."""

import os
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('save_every_n', 5)
        self.declare_parameter('max_images', 500)
        self.declare_parameter('image_format', 'png')
        # use_sim_time is set by the launch file — do not declare it again (Jazzy)

        topic = self.get_parameter('image_topic').get_parameter_value().string_value
        out = self.get_parameter('output_dir').get_parameter_value().string_value
        self.save_every_n = max(
            1, self.get_parameter('save_every_n').get_parameter_value().integer_value
        )
        self.max_images = self.get_parameter('max_images').get_parameter_value().integer_value
        self.image_format = (
            self.get_parameter('image_format').get_parameter_value().string_value or 'png'
        )

        if not out:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            out = os.path.expanduser(f'~/ugv_vision_nav_ws/data/images_{stamp}')

        self.output_dir = os.path.expanduser(out)
        os.makedirs(self.output_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.saved_count = 0
        self._max_logged = False

        self.sub = self.create_subscription(Image, topic, self._on_image, 10)

        self.get_logger().info(f'Saving every {self.save_every_n} frame(s) from [{topic}]')
        self.get_logger().info(f'Output directory: {self.output_dir}')
        self.get_logger().info(f'Will stop after {self.max_images} images (0 = unlimited)')

    def _on_image(self, msg: Image) -> None:
        self.frame_count += 1
        if self.frame_count % self.save_every_n != 0:
            return
        if self.max_images > 0 and self.saved_count >= self.max_images:
            if not self._max_logged:
                self.get_logger().info(
                    f'Reached max_images={self.max_images}. Not saving more.'
                )
                self._max_logged = True
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge error: {exc}')
            return

        name = f'frame_{self.saved_count:05d}.{self.image_format}'
        path = os.path.join(self.output_dir, name)
        ok = cv2.imwrite(path, cv_img)
        if not ok:
            self.get_logger().error(f'Failed to write {path}')
            return

        self.saved_count += 1
        if self.saved_count == 1 or self.saved_count % 25 == 0:
            self.get_logger().info(f'Saved {self.saved_count} images → {path}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Done. Saved {min(node.saved_count, node.max_images if node.max_images > 0 else node.saved_count)} '
            f'images under {node.output_dir}'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
