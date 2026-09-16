#!/usr/bin/env python3
"""
Example edge processor: subscribe to the sensor bus, do light work, publish results.

This is the slot where real perception (ground seg, rock detect, etc.) goes later.
It never talks to StreamHub or the browser — only bus topics.
"""

from __future__ import annotations

import json
import struct
from typing import Any, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from edge_sim.topics import SensorBusContract


def _decode_xyz(msg: PointCloud2) -> np.ndarray:
    offsets = {f.name: f.offset for f in msg.fields}
    if not {'x', 'y', 'z'}.issubset(offsets):
        return np.zeros((0, 3), dtype=np.float32)
    step = int(msg.point_step)
    n = int(msg.width) * int(msg.height)
    data = memoryview(msg.data)
    ox, oy, oz = offsets['x'], offsets['y'], offsets['z']
    pts = np.zeros((n, 3), dtype=np.float32)
    valid = 0
    for i in range(n):
        base = i * step
        if base + 12 > len(data):
            break
        x = struct.unpack_from('<f', data, base + ox)[0]
        y = struct.unpack_from('<f', data, base + oy)[0]
        z = struct.unpack_from('<f', data, base + oz)[0]
        pts[valid, 0] = x
        pts[valid, 1] = y
        pts[valid, 2] = z
        valid += 1
    return pts[:valid]


def _encode_xyz(points: np.ndarray, header: Any) -> PointCloud2:
    n = int(points.shape[0])
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.is_bigendian = False
    msg.is_dense = True
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16
    msg.row_step = 16 * n
    buf = bytearray(msg.row_step)
    for i in range(n):
        # simple height-based "intensity" as placeholder feature
        inten = float(np.clip(0.5 + 0.2 * points[i, 2], 0.0, 1.0))
        struct.pack_into('<ffff', buf, i * 16, float(points[i, 0]), float(points[i, 1]), float(points[i, 2]), inten)
    msg.data = bytes(buf)
    return msg


class ProcessorStubNode(Node):
    """
    Pass-through-ish processor:
      /lidar → downsample + z-band filter → /edge/lidar/processed
      also publishes /edge/status JSON heartbeat
    """

    def __init__(self) -> None:
        super().__init__('edge_processor')
        c = SensorBusContract()
        self.declare_parameter('lidar_topic', c.lidar_topic)
        self.declare_parameter('processed_lidar_topic', c.processed_lidar_topic)
        self.declare_parameter('edge_status_topic', c.edge_status_topic)
        self.declare_parameter('max_points', 2000)
        self.declare_parameter('z_min', -1.0)
        self.declare_parameter('z_max', 5.0)

        lidar_topic = str(self.get_parameter('lidar_topic').value)
        out_topic = str(self.get_parameter('processed_lidar_topic').value)
        status_topic = str(self.get_parameter('edge_status_topic').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)

        self._frames = 0
        self.pub_cloud = self.create_publisher(PointCloud2, out_topic, qos_profile_sensor_data)
        self.pub_status = self.create_publisher(String, status_topic, 10)
        self.create_subscription(PointCloud2, lidar_topic, self._on_lidar, qos_profile_sensor_data)
        self.create_timer(1.0, self._on_heartbeat)

        self.get_logger().info(
            f'edge_processor  {lidar_topic} → {out_topic}  (z in [{self.z_min},{self.z_max}])'
        )

    def _on_lidar(self, msg: PointCloud2) -> None:
        pts = _decode_xyz(msg)
        if pts.size == 0:
            return
        mask = (pts[:, 2] >= self.z_min) & (pts[:, 2] <= self.z_max)
        pts = pts[mask]
        if pts.shape[0] > self.max_points:
            idx = np.linspace(0, pts.shape[0] - 1, self.max_points).astype(int)
            pts = pts[idx]
        out = _encode_xyz(pts, msg.header)
        self.pub_cloud.publish(out)
        self._frames += 1

    def _on_heartbeat(self) -> None:
        payload = {
            'node': 'edge_processor',
            'frames': self._frames,
            'z_band': [self.z_min, self.z_max],
            'max_points': self.max_points,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.pub_status.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ProcessorStubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
