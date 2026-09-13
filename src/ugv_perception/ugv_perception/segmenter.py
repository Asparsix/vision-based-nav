#!/usr/bin/env python3
"""Live UNet segmentation from camera topic."""

from __future__ import annotations

import os
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image

from ugv_perception.unet import UNet

try:
    import torch
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        'PyTorch not found. Source the workspace venv site-packages, e.g.\n'
        '  export PYTHONPATH=~/ugv_vision_nav_ws/.venv/lib/python3.12/site-packages:$PYTHONPATH\n'
        'Or use: ros2 launch ugv_perception segment.launch.py'
    ) from exc


CLASS_NAMES = ['background', 'road', 'grass', 'tree']
# Bright BGR colors so rqt is obvious (not near-black class IDs).
PREVIEW_BGR = {
    0: (220, 220, 220),  # background / sky — light gray
    1: (0, 165, 255),    # road — orange
    2: (0, 255, 0),      # grass — bright green
    3: (0, 0, 255),      # tree — red
}


def colorize(mask: np.ndarray) -> np.ndarray:
    out = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
    for cid, bgr in PREVIEW_BGR.items():
        out[mask == cid] = bgr
    return out


def numpy_to_imgmsg(arr: np.ndarray, encoding: str, header) -> Image:
    """Build Image without cv_bridge type maps (avoids venv OpenCV clash)."""
    msg = Image()
    msg.header = header
    msg.height = int(arr.shape[0])
    msg.width = int(arr.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0
    if arr.ndim == 2:
        msg.step = msg.width
    else:
        msg.step = msg.width * int(arr.shape[2])
    msg.data = np.ascontiguousarray(arr).tobytes()
    return msg


class Segmenter(Node):
    def __init__(self) -> None:
        super().__init__('ugv_segmenter')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter(
            'weights',
            str(Path.home() / 'ugv_vision_nav_ws/data/models/unet_outdoor_best.pt'),
        )
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('publish_overlay', True)
        self.declare_parameter('overlay_alpha', 0.55)
        self.declare_parameter('skip_frames', 0)

        topic = self.get_parameter('image_topic').value
        weights = os.path.expanduser(str(self.get_parameter('weights').value))
        device_name = str(self.get_parameter('device').value)
        self.publish_overlay = bool(self.get_parameter('publish_overlay').value)
        self.overlay_alpha = float(self.get_parameter('overlay_alpha').value)
        self.skip_frames = max(0, int(self.get_parameter('skip_frames').value))

        if not os.path.isfile(weights):
            raise FileNotFoundError(f'Weights not found: {weights}')

        self.device = torch.device(device_name)
        ckpt = torch.load(weights, map_location=self.device, weights_only=False)
        num_classes = int(ckpt.get('num_classes', 4))
        base = int(ckpt.get('base', 32))
        size = ckpt.get('size', [320, 240])
        self.infer_w, self.infer_h = int(size[0]), int(size[1])

        self.model = UNet(num_classes=num_classes, base=base).to(self.device)
        self.model.load_state_dict(ckpt['model'])
        self.model.eval()

        self.bridge = CvBridge()
        self._frame_i = 0
        self._infer_count = 0

        self.sub = self.create_subscription(Image, topic, self._on_image, 10)
        self.pub_mask = self.create_publisher(Image, '/perception/segmentation', 10)
        self.pub_mask_vis = self.create_publisher(Image, '/perception/segmentation_vis', 10)
        self.pub_color = self.create_publisher(Image, '/perception/segmentation_color', 10)
        self.pub_overlay = self.create_publisher(Image, '/perception/segmentation_overlay', 10)

        miou = ckpt.get('val_miou', None)
        miou_s = f'{float(miou):.3f}' if miou is not None else 'n/a'
        self.get_logger().info(
            f'Loaded {weights} (epoch={ckpt.get("epoch", "?")} mIoU={miou_s} '
            f'size={self.infer_w}x{self.infer_h} device={self.device})'
        )
        self.get_logger().info(f'Subscribed to [{topic}]  classes={CLASS_NAMES}')
        self.get_logger().info(
            'Publishing /perception/segmentation (dark class IDs — looks black), '
            '/perception/segmentation_vis (bright), '
            '/perception/segmentation_color , /perception/segmentation_overlay'
        )

    @torch.no_grad()
    def _predict(self, bgr: np.ndarray) -> np.ndarray:
        h, w = bgr.shape[:2]
        resized = cv2.resize(bgr, (self.infer_w, self.infer_h), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        x = torch.from_numpy(rgb).permute(2, 0, 1).unsqueeze(0).to(self.device)
        pred = self.model(x).argmax(dim=1).squeeze(0).cpu().numpy().astype(np.uint8)
        return cv2.resize(pred, (w, h), interpolation=cv2.INTER_NEAREST)

    def _on_image(self, msg: Image) -> None:
        self._frame_i += 1
        if self.skip_frames > 0 and (self._frame_i % (self.skip_frames + 1)) != 0:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge error: {exc}')
            return

        mask = self._predict(bgr)
        color = colorize(mask)
        # Scale class IDs so a mono viewer is not pure black (0,85,170,255).
        mask_vis = (mask.astype(np.uint16) * 85).clip(0, 255).astype(np.uint8)

        self.pub_mask.publish(numpy_to_imgmsg(mask, 'mono8', msg.header))
        self.pub_mask_vis.publish(numpy_to_imgmsg(mask_vis, 'mono8', msg.header))
        # Publish color/overlay as rgb8 (matches Gazebo camera + most viewers).
        color_rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        self.pub_color.publish(numpy_to_imgmsg(color_rgb, 'rgb8', msg.header))

        if self.publish_overlay:
            overlay = cv2.addWeighted(bgr, 1.0 - self.overlay_alpha, color, self.overlay_alpha, 0)
            overlay_rgb = cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB)
            self.pub_overlay.publish(numpy_to_imgmsg(overlay_rgb, 'rgb8', msg.header))

        self._infer_count += 1
        if self._infer_count == 1 or self._infer_count % 50 == 0:
            self.get_logger().info(f'Inferred {self._infer_count} frames')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Segmenter()
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
