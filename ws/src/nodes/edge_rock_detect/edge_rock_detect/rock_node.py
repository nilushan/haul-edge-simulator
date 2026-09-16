#!/usr/bin/env python3
"""LiDAR rock detector: /lidar → clouds + detections + alerts."""

from __future__ import annotations

import json
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from edge_perception.cloud_io import decode_xyz, encode_xyz_label
from edge_perception.geometry import roi_mask, voxel_downsample
from edge_perception.ground import estimate_ground_grid
from edge_perception.rocks import RockParams, detect_rocks
from edge_perception.schema import AlertEvent, DetectionSet, Label, Severity
from edge_sim.topics import SensorBusContract


class RockDetectNode(Node):
    def __init__(self) -> None:
        super().__init__('edge_rock_detect')
        c = SensorBusContract()
        self.declare_parameter('lidar_topic', c.lidar_topic)
        self.declare_parameter('rocks_cloud_topic', c.rocks_cloud_topic)
        self.declare_parameter('obstacles_cloud_topic', c.obstacles_cloud_topic)
        self.declare_parameter('ground_cloud_topic', c.ground_cloud_topic)
        self.declare_parameter('detections_topic', c.detections_topic)
        self.declare_parameter('alerts_topic', c.alerts_topic)
        self.declare_parameter('status_topic', c.edge_status_topic)
        self.declare_parameter('vehicle_id', 'haul-01')
        self.declare_parameter('alert_min_conf', 0.7)
        self.declare_parameter('publish_ground', True)

        self.vehicle_id = str(self.get_parameter('vehicle_id').value)
        self.alert_min_conf = float(self.get_parameter('alert_min_conf').value)
        self.publish_ground = bool(self.get_parameter('publish_ground').value)
        self._params = RockParams()
        self._frames = 0
        self._alerts = 0
        self._seen_keys: set[str] = set()

        lidar = str(self.get_parameter('lidar_topic').value)
        self.pub_rocks = self.create_publisher(
            PointCloud2, str(self.get_parameter('rocks_cloud_topic').value), qos_profile_sensor_data
        )
        self.pub_obs = self.create_publisher(
            PointCloud2, str(self.get_parameter('obstacles_cloud_topic').value), qos_profile_sensor_data
        )
        self.pub_ground = self.create_publisher(
            PointCloud2, str(self.get_parameter('ground_cloud_topic').value), qos_profile_sensor_data
        )
        self.pub_det = self.create_publisher(String, str(self.get_parameter('detections_topic').value), 10)
        self.pub_alert = self.create_publisher(String, str(self.get_parameter('alerts_topic').value), 10)
        self.pub_status = self.create_publisher(String, str(self.get_parameter('status_topic').value), 10)
        self.create_subscription(PointCloud2, lidar, self._on_lidar, qos_profile_sensor_data)
        self.create_timer(2.0, self._heartbeat)
        self.get_logger().info(f'edge_rock_detect listening on {lidar}')

    def _on_lidar(self, msg: PointCloud2) -> None:
        pts, _inten = decode_xyz(msg)
        if pts.size == 0:
            return
        mask = roi_mask(pts)
        pts = pts[mask]
        pts = voxel_downsample(pts, 0.15)
        if pts.shape[0] < 10:
            return

        gr = estimate_ground_grid(pts)
        ground = pts[gr.ground_idx]
        obs = pts[gr.obstacle_idx]
        hag_obs = gr.hag[gr.obstacle_idx]

        if self.publish_ground and ground.shape[0]:
            labels = np.full((ground.shape[0],), float(Label.GROUND), dtype=np.float32)
            self.pub_ground.publish(encode_xyz_label(ground, msg.header, labels=labels))

        if obs.shape[0]:
            labels = np.full((obs.shape[0],), float(Label.OBSTACLE), dtype=np.float32)
            self.pub_obs.publish(encode_xyz_label(obs, msg.header, labels=labels))

        # Ignore near-field / body-locked ghosts (cab, hood) — only real ahead rocks
        dets = [
            d
            for d in detect_rocks(obs, hag_obs, params=self._params)
            if d.x >= 8.0 and abs(d.y) <= 6.0 and d.confidence >= 0.6
        ]
        rock_pts = []
        rock_labels = []
        rock_conf = []
        for d in dets:
            # approximate rock points: near centroid within radius
            if obs.shape[0] == 0:
                break
            dist = np.linalg.norm(obs - np.array([d.x, d.y, d.z], dtype=np.float32), axis=1)
            sel = dist <= max(d.radius_m * 1.2, 0.3)
            if np.any(sel):
                rock_pts.append(obs[sel])
                n = int(sel.sum())
                rock_labels.append(np.full((n,), float(Label.ROCK), dtype=np.float32))
                rock_conf.append(np.full((n,), float(d.confidence), dtype=np.float32))

        if rock_pts:
            cloud = np.vstack(rock_pts)
            self.pub_rocks.publish(
                encode_xyz_label(
                    cloud,
                    msg.header,
                    labels=np.concatenate(rock_labels),
                    conf=np.concatenate(rock_conf),
                )
            )

        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        dset = DetectionSet(t=t, frame_id=msg.header.frame_id or 'base_link', source_node='edge_rock_detect', detections=dets)
        s = String()
        s.data = json.dumps(dset.to_dict())
        self.pub_det.publish(s)

        for d in dets:
            if d.confidence < self.alert_min_conf:
                continue
            # spatial dedup key (coarse grid)
            key = f'{int(d.x // 2)}:{int(d.y // 2)}'
            if key in self._seen_keys:
                continue
            self._seen_keys.add(key)
            if len(self._seen_keys) > 200:
                self._seen_keys = set(list(self._seen_keys)[-100:])
            sev = Severity.WARN if d.radius_m >= 0.7 or abs(d.y) < 3.0 else Severity.INFO
            if d.details.get('range_m', 99) < 10 and abs(d.y) < 2.5:
                sev = Severity.CRITICAL
            alert = AlertEvent(
                type='rock',
                severity=sev,
                confidence=d.confidence,
                source_node='edge_rock_detect',
                vehicle_id=self.vehicle_id,
                frame_id=d.frame_id,
                t_ros=t,
                t_vehicle=t,
                x=d.x,
                y=d.y,
                z=d.z,
                geometry={'kind': 'sphere', 'radius_m': d.radius_m, 'extent_m': list(d.extent_m)},
                details=d.details,
                cloud_ref={'topic': SensorBusContract().rocks_cloud_topic, 'stamp': t},
            )
            a = String()
            a.data = json.dumps(alert.to_dict())
            self.pub_alert.publish(a)
            self._alerts += 1

        self._frames += 1

    def _heartbeat(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                'node': 'edge_rock_detect',
                'frames': self._frames,
                'alerts': self._alerts,
                'tracks_cached': len(self._seen_keys),
            }
        )
        self.pub_status.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = RockDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
