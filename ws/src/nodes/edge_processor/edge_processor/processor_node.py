#!/usr/bin/env python3
"""
Example edge processor: subscribe to the sensor bus, do light work, publish results.

This is the slot where real perception (ground seg, rock detect, etc.) goes later.
It never talks to StreamHub or the browser — only bus topics.
"""

from __future__ import annotations

import json
from typing import Any, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from edge_sim.topics import SensorBusContract
from edge_processor.cloud_ops import decode_xyz, encode_processed, process_points


def _decode_xyz(msg: PointCloud2) -> np.ndarray:
    return decode_xyz(msg)[0]


def _encode_xyz(points: np.ndarray, header: Any) -> PointCloud2:
    return encode_processed(points, header)  # type: ignore[return-value]


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
        if self.max_points < 1 or not np.isfinite(self.z_min) or not np.isfinite(self.z_max) or self.z_min > self.z_max:
            raise ValueError('require max_points > 0 and finite z_min <= z_max')

        self._frames = 0
        self._bad_frames = 0
        self.pub_cloud = self.create_publisher(PointCloud2, out_topic, qos_profile_sensor_data)
        self.pub_status = self.create_publisher(String, status_topic, 10)
        self.create_subscription(PointCloud2, lidar_topic, self._on_lidar, qos_profile_sensor_data)
        self.create_timer(1.0, self._on_heartbeat)

        self.get_logger().info(
            f'edge_processor  {lidar_topic} → {out_topic}  (z in [{self.z_min},{self.z_max}])'
        )

    def _on_lidar(self, msg: PointCloud2) -> None:
        try:
            points = _decode_xyz(msg)
            if points.size == 0:
                return
            points = process_points(
                points,
                z_min=self.z_min,
                z_max=self.z_max,
                max_points=self.max_points,
            )
            self.pub_cloud.publish(_encode_xyz(points, msg.header))
            self._frames += 1
        except (ValueError, TypeError, IndexError) as exc:
            self._bad_frames += 1
            self.get_logger().warn(f'ignoring malformed lidar frame: {exc}')

    def _on_heartbeat(self) -> None:
        payload = {
            'node': 'edge_processor',
            'frames': self._frames,
            'z_band': [self.z_min, self.z_max],
            'max_points': self.max_points,
            'bad_frames': self._bad_frames,
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
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
