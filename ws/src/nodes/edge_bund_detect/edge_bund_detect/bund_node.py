#!/usr/bin/env python3
"""Bund/berm detector: /lidar → /edge/lidar/bunds + alerts."""

from __future__ import annotations

import json
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from edge_perception.bunds import BundParams, bund_points_from_detections, detect_bunds
from edge_perception.cloud_io import decode_xyz, encode_xyz_label
from edge_perception.geometry import roi_mask, voxel_downsample
from edge_perception.schema import AlertEvent, DetectionSet, Label, Severity
from edge_sim.topics import SensorBusContract


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
        self._params = BundParams()
        self._frames = 0
        self._alerts = 0
        self._last_alert_t = -1e9

        lidar = str(self.get_parameter('lidar_topic').value)
        self.pub_cloud = self.create_publisher(
            PointCloud2, str(self.get_parameter('bunds_cloud_topic').value), qos_profile_sensor_data
        )
        self.pub_det = self.create_publisher(String, str(self.get_parameter('detections_topic').value), 10)
        self.pub_alert = self.create_publisher(String, str(self.get_parameter('alerts_topic').value), 10)
        self.pub_status = self.create_publisher(String, str(self.get_parameter('status_topic').value), 10)
        self.create_subscription(PointCloud2, lidar, self._on_lidar, qos_profile_sensor_data)
        self.create_timer(2.0, self._heartbeat)
        self.get_logger().info(f'edge_bund_detect listening on {lidar}')

    def _on_lidar(self, msg: PointCloud2) -> None:
        pts, _ = decode_xyz(msg)
        if pts.size == 0:
            return
        pts = pts[roi_mask(pts, y_abs_max=12.0)]
        pts = voxel_downsample(pts, 0.2)
        if pts.shape[0] < 20:
            return

        dets, alert_specs = detect_bunds(pts, self._params)
        bund_pts = bund_points_from_detections(pts, self._params)
        if bund_pts.shape[0]:
            labels = np.full((bund_pts.shape[0],), float(Label.BUND), dtype=np.float32)
            self.pub_cloud.publish(encode_xyz_label(bund_pts, msg.header, labels=labels))

        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        dset = DetectionSet(
            t=t,
            frame_id=msg.header.frame_id or 'base_link',
            source_node='edge_bund_detect',
            detections=dets,
        )
        s = String()
        s.data = json.dumps(dset.to_dict())
        self.pub_det.publish(s)

        if alert_specs and (t - self._last_alert_t) >= self.cooldown:
            for spec in alert_specs:
                pose = next((d for d in dets if d.details.get('side') == spec.get('side')), None)
                alert = AlertEvent(
                    type=str(spec.get('type', 'bund_gap')),
                    severity=str(spec.get('severity', Severity.WARN)),
                    confidence=0.75,
                    source_node='edge_bund_detect',
                    vehicle_id=self.vehicle_id,
                    t_ros=t,
                    t_vehicle=t,
                    x=pose.x if pose else 0.0,
                    y=pose.y if pose else 0.0,
                    z=pose.z if pose else 0.0,
                    details=spec,
                    cloud_ref={'topic': SensorBusContract().bunds_cloud_topic, 'stamp': t},
                )
                a = String()
                a.data = json.dumps(alert.to_dict())
                self.pub_alert.publish(a)
                self._alerts += 1
            self._last_alert_t = t

        self._frames += 1

    def _heartbeat(self) -> None:
        msg = String()
        msg.data = json.dumps({'node': 'edge_bund_detect', 'frames': self._frames, 'alerts': self._alerts})
        self.pub_status.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = BundDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
