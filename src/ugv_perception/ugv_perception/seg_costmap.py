#!/usr/bin/env python3
"""Project camera segmentation into a local OccupancyGrid costmap.

Classes (from UNet):
  0 background → unknown (skip)
  1 road       → free (0)
  2 grass      → medium cost
  3 tree       → lethal (100)

Uses a flat-ground ray cast with the UGV camera extrinsics from ugv_bot.urdf.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
import struct

CLASS_BG = 0
CLASS_ROAD = 1
CLASS_GRASS = 2
CLASS_TREE = 3


def rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Intrinsic XYZ fixed-axis rotation (URDF rpy)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    return rz @ ry @ rx


class SegCostmap(Node):
    def __init__(self) -> None:
        super().__init__('ugv_seg_costmap')

        self.declare_parameter('mask_topic', '/perception/segmentation')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('output_topic', '/perception/local_costmap')
        self.declare_parameter('obstacles_topic', '/perception/semantic_obstacles')
        self.declare_parameter('frame_id', 'base_link')
        # Camera pose: left stereo camera in base_link
        self.declare_parameter('cam_x', 0.28)
        self.declare_parameter('cam_y', 0.04)
        self.declare_parameter('cam_z', 0.08)  # camera_link z in base_link
        self.declare_parameter('cam_pitch', 0.12)
        self.declare_parameter('ground_z', -0.10)  # ground plane in base_link
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('x_min', 0.30)
        self.declare_parameter('x_max', 6.0)
        self.declare_parameter('y_min', -2.5)
        self.declare_parameter('y_max', 2.5)
        self.declare_parameter('grass_cost', 45)
        self.declare_parameter('road_cost', 0)
        self.declare_parameter('tree_cost', 100)
        self.declare_parameter('tree_inflate_cells', 2)
        self.declare_parameter('stride', 2)  # subsample pixels for CPU

        self.mask_topic = str(self.get_parameter('mask_topic').value)
        self.out_topic = str(self.get_parameter('output_topic').value)
        self.obstacles_topic = str(self.get_parameter('obstacles_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.cam_xyz = np.array(
            [
                float(self.get_parameter('cam_x').value),
                float(self.get_parameter('cam_y').value),
                float(self.get_parameter('cam_z').value),
            ],
            dtype=np.float64,
        )
        self.ground_z = float(self.get_parameter('ground_z').value)
        pitch = float(self.get_parameter('cam_pitch').value)
        # optical -> camera_link (inverse of optical joint rpy -90,0,-90)
        r_link_from_optical = rpy_to_rot(-math.pi / 2.0, 0.0, -math.pi / 2.0)
        r_base_from_link = rpy_to_rot(0.0, pitch, 0.0)
        self.r_base_from_optical = r_base_from_link @ r_link_from_optical

        self.resolution = float(self.get_parameter('resolution').value)
        self.x_min = float(self.get_parameter('x_min').value)
        self.x_max = float(self.get_parameter('x_max').value)
        self.y_min = float(self.get_parameter('y_min').value)
        self.y_max = float(self.get_parameter('y_max').value)
        self.grass_cost = int(self.get_parameter('grass_cost').value)
        self.road_cost = int(self.get_parameter('road_cost').value)
        self.tree_cost = int(self.get_parameter('tree_cost').value)
        self.tree_inflate = max(0, int(self.get_parameter('tree_inflate_cells').value))
        self.stride = max(1, int(self.get_parameter('stride').value))

        self.width = int(round((self.x_max - self.x_min) / self.resolution))
        self.height = int(round((self.y_max - self.y_min) / self.resolution))

        self.fx = self.fy = self.cx = self.cy = None
        self._pub_count = 0

        self.pub = self.create_publisher(OccupancyGrid, self.out_topic, 10)
        self.pub_cloud = self.create_publisher(PointCloud2, self.obstacles_topic, 10)
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._on_info,
            10,
        )
        self.create_subscription(Image, self.mask_topic, self._on_mask, 10)

        self.get_logger().info(
            f'Costmap {self.width}x{self.height} @ {self.resolution}m '
            f'x=[{self.x_min},{self.x_max}] y=[{self.y_min},{self.y_max}] '
            f'frame={self.frame_id}'
        )
        self.get_logger().info(
            f'Sub [{self.mask_topic}] → [{self.out_topic}] + [{self.obstacles_topic}] '
            f'(road={self.road_cost} grass={self.grass_cost} tree={self.tree_cost})'
        )

    def _on_info(self, msg: CameraInfo) -> None:
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def _class_to_cost(self, cls: int) -> int | None:
        if cls == CLASS_ROAD:
            return self.road_cost
        if cls == CLASS_GRASS:
            return self.grass_cost
        if cls == CLASS_TREE:
            return self.tree_cost
        return None  # background / unknown → do not write

    def _on_mask(self, msg: Image) -> None:
        if self.fx is None:
            return
        if msg.encoding not in ('mono8', '8UC1'):
            self.get_logger().warn(f'Unexpected mask encoding {msg.encoding}')
            return

        h, w = int(msg.height), int(msg.width)
        mask = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if mask.size < h * w:
            return
        mask = mask[: h * w].reshape(h, w)

        # OccupancyGrid: row-major, index = y_cell * width + x_cell
        # Start unknown
        grid = np.full((self.height, self.width), -1, dtype=np.int8)

        # Only use lower image (ground); skip sky band
        v0 = int(h * 0.35)
        us = np.arange(0, w, self.stride)
        vs = np.arange(v0, h, self.stride)
        uu, vv = np.meshgrid(us, vs)
        uu = uu.reshape(-1).astype(np.float64)
        vv = vv.reshape(-1).astype(np.float64)
        cls = mask[vv.astype(np.int32), uu.astype(np.int32)].astype(np.int32)

        # Rays in optical frame
        x_o = (uu - self.cx) / self.fx
        y_o = (vv - self.cy) / self.fy
        z_o = np.ones_like(x_o)
        dirs_o = np.stack([x_o, y_o, z_o], axis=0)  # 3xN
        dirs_b = self.r_base_from_optical @ dirs_o  # 3xN in base/footprint-ish

        # Intersect ground plane z=ground_z: p = cam + t * d
        dz = dirs_b[2]
        valid = np.abs(dz) > 1e-5
        t = np.zeros_like(dz)
        t[valid] = (self.ground_z - self.cam_xyz[2]) / dz[valid]
        valid &= t > 0.05
        px = self.cam_xyz[0] + t * dirs_b[0]
        py = self.cam_xyz[1] + t * dirs_b[1]
        # Prefer rays that hit in front of the robot
        valid &= (px >= self.x_min) & (px < self.x_max) & (py >= self.y_min) & (py < self.y_max)

        ix = ((px - self.x_min) / self.resolution).astype(np.int32)
        iy = ((py - self.y_min) / self.resolution).astype(np.int32)
        valid &= (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)

        ix = ix[valid]
        iy = iy[valid]
        cls = cls[valid]

        # Write costs; lethal / higher cost wins
        for c, x, y in zip(cls, ix, iy):
            cost = self._class_to_cost(int(c))
            if cost is None:
                continue
            cur = int(grid[y, x])
            if cur < 0 or cost > cur:
                grid[y, x] = cost

        if self.tree_inflate > 0:
            tree = grid == np.int8(self.tree_cost)
            if tree.any():
                inflated = tree.copy()
                r = self.tree_inflate
                # square inflate without OpenCV
                yy, xx = np.where(tree)
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        if dx * dx + dy * dy > r * r + r:
                            continue
                        y2 = yy + dy
                        x2 = xx + dx
                        ok = (y2 >= 0) & (y2 < self.height) & (x2 >= 0) & (x2 < self.width)
                        inflated[y2[ok], x2[ok]] = True
                grid[inflated & (grid < self.tree_cost)] = np.int8(self.tree_cost)

        out = OccupancyGrid()
        out.header = Header()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.info.resolution = self.resolution
        out.info.width = self.width
        out.info.height = self.height
        # origin: cell (0,0) lower-left corner in frame
        out.info.origin.position.x = self.x_min
        out.info.origin.position.y = self.y_min
        out.info.origin.position.z = 0.0
        out.info.origin.orientation.w = 1.0
        out.data = grid.reshape(-1).astype(np.int8).tolist()
        self.pub.publish(out)

        # PointCloud of non-road cells for Nav2 obstacle layer (trees + grass)
        yy, xx = np.where(grid >= max(1, self.grass_cost))
        if yy.size > 0:
            xs = self.x_min + (xx.astype(np.float32) + 0.5) * self.resolution
            ys = self.y_min + (yy.astype(np.float32) + 0.5) * self.resolution
            zs = np.zeros_like(xs)
            # Mark trees taller so inflation treats them as solid
            tree_mask = grid[yy, xx] >= 90
            zs[tree_mask] = 0.35
            cloud = PointCloud2()
            cloud.header = out.header
            cloud.height = 1
            cloud.width = int(xs.size)
            cloud.is_dense = True
            cloud.is_bigendian = False
            cloud.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            cloud.point_step = 12
            cloud.row_step = cloud.point_step * cloud.width
            buf = b''.join(
                struct.pack('<fff', float(x), float(y), float(z))
                for x, y, z in zip(xs, ys, zs)
            )
            cloud.data = buf
            self.pub_cloud.publish(cloud)

        self._pub_count += 1
        if self._pub_count == 1 or self._pub_count % 50 == 0:
            known = int(np.sum(grid >= 0))
            lethal = int(np.sum(grid >= 90))
            self.get_logger().info(
                f'Published costmap #{self._pub_count} known={known} lethal≈{lethal}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SegCostmap()
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
