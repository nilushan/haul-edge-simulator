#!/usr/bin/env python3
"""Bund/berm detector: /lidar → /edge/lidar/bunds + alerts."""

from __future__ import annotations

import json
import time
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from edge_perception.cloud_io import decode_xyz, encode_xyz_label
from edge_perception.geometry import roi_mask, voxel_downsample
from edge_perception.schema import DetectionSet, Label
from edge_perception.semantics import SemanticParams, classify_frame
from edge_sim.topics import SensorBusContract
from edge_bund_detect.alerting import CooldownGate


class BundDetectNode(Node):
    def __init__(self) -> None:
        super().__init__('edge_bund_detect')
        c = SensorBusContract()
        self.declare_parameter('lidar_topic', c.lidar_topic)
        self.declare_parameter('bunds_cloud_topic', c.bunds_cloud_topic)
        self.declare_parameter('detections_topic', c.detections_topic)
        self.declare_parameter('alerts_topic', c.alerts_topic)
        self.declare_parameter('status_topic', c.edge_status_topic)
        self.declare_parameter('vehicle_id', 'haul-01')
        self.declare_parameter('alert_cooldown_s', 5.0)

        self.vehicle_id = str(self.get_parameter('vehicle_id').value)
        self.cooldown = float(self.get_parameter('alert_cooldown_s').value)
        self._gate = CooldownGate(self.cooldown)
        self._params = SemanticParams()
        self._frames = 0
        self._alerts = 0
        self._bad_frames = 0

        lidar = str(self.get_parameter('lidar_topic').value)
        self._bunds_cloud_topic = str(self.get_parameter('bunds_cloud_topic').value)
        self.pub_cloud = self.create_publisher(
            PointCloud2, self._bunds_cloud_topic, qos_profile_sensor_data
        )
        self.pub_det = self.create_publisher(String, str(self.get_parameter('detections_topic').value), 10)
        self.pub_alert = self.create_publisher(String, str(self.get_parameter('alerts_topic').value), 10)
        self.pub_status = self.create_publisher(String, str(self.get_parameter('status_topic').value), 10)
        self.create_subscription(PointCloud2, lidar, self._on_lidar, qos_profile_sensor_data)
        self.create_timer(2.0, self._heartbeat)
        self.get_logger().info(f'edge_bund_detect listening on {lidar}')

    def _on_lidar(self, msg: PointCloud2) -> None:
        try:
            self._process_lidar(msg)
        except (ValueError, TypeError, IndexError) as exc:
            self._bad_frames += 1
            self.get_logger().warn(f'ignoring malformed lidar frame: {exc}')

    def _process_lidar(self, msg: PointCloud2) -> None:
        pts, _ = decode_xyz(msg)
        if pts.size == 0:
            return
        pts = pts[roi_mask(pts)]
        pts = voxel_downsample(pts, 0.2)
        if pts.shape[0] < 20:
            return

        sensor_t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        ros_now = float(self.get_clock().now().nanoseconds) * 1e-9
        t_ros = sensor_t if sensor_t > 0.0 else ros_now
        input_frame = msg.header.frame_id or 'base_link'

        result = classify_frame(
            pts,
            params=self._params,
            t=sensor_t,
            frame_id=input_frame,
            vehicle_id=self.vehicle_id,
            source_node='edge_bund_detect',
        )
        # Crest points carry their own class, so an under-height section is
        # distinguishable from a compliant one on the cloud alone.
        bund_mask = np.isin(result.labels, [int(Label.BUND), int(Label.BUND_LOW)])
        if np.any(bund_mask):
            self.pub_cloud.publish(
                encode_xyz_label(
                    pts[bund_mask],
                    msg.header,
                    labels=result.labels[bund_mask].astype(np.float32),
                    conf=result.conf[bund_mask],
                )
            )

        dets = [d for d in result.detections if d.type.startswith('bund')]
        dset = DetectionSet(
            t=sensor_t,
            frame_id=input_frame,
            source_node='edge_bund_detect',
            detections=dets,
        )
        s = String()
        s.data = json.dumps(dset.to_dict())
        self.pub_det.publish(s)

        events = [e for e in result.events if e.type.startswith('bund')]
        if events and self._gate.allow(time.monotonic()):
            for alert in events:
                alert.t_ros = t_ros
                alert.cloud_ref = {'topic': self._bunds_cloud_topic, 'stamp': sensor_t}
                a = String()
                a.data = json.dumps(alert.to_dict())
                self.pub_alert.publish(a)
                self._alerts += 1

        self._frames += 1

    def _heartbeat(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                'node': 'edge_bund_detect',
                'frames': self._frames,
                'alerts': self._alerts,
                'bad_frames': self._bad_frames,
            }
        )
        self.pub_status.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = BundDetectNode()
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
