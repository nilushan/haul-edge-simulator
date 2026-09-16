#!/usr/bin/env python3
"""LiDAR rock detector: /lidar → clouds + detections + alerts."""

from __future__ import annotations

import json
import math
import time
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from edge_perception.cloud_io import decode_xyz, encode_xyz_label
from edge_perception.geometry import roi_mask, voxel_downsample
from edge_perception.ground import estimate_ground_grid
from edge_perception.rocks import RockParams, detect_rocks
from edge_perception.schema import AlertEvent, DetectionSet, Label, Severity
from edge_sim.topics import SensorBusContract
from edge_rock_detect.tracking import Pose2D, SpatialDeduplicator, body_to_map


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
        self.declare_parameter('odom_topic', c.odom_topic)
        self.declare_parameter('odom_timeout_s', 1.0)
        self.declare_parameter('dedup_cell_m', 2.0)
        self.declare_parameter('dedup_ttl_s', 120.0)
        self.declare_parameter('max_cluster_points', 2000)
        self.declare_parameter('frame_map', c.frame_map)

        self.vehicle_id = str(self.get_parameter('vehicle_id').value)
        self.alert_min_conf = float(self.get_parameter('alert_min_conf').value)
        self.publish_ground = bool(self.get_parameter('publish_ground').value)
        self.frame_map = str(self.get_parameter('frame_map').value)
        self._params = RockParams()
        self._frames = 0
        self._alerts = 0
        self._bad_frames = 0
        self._pose: Optional[Pose2D] = None
        self._pose_received_at = -1e9
        self._odom_timeout_s = float(self.get_parameter('odom_timeout_s').value)
        self._max_cluster_points = int(self.get_parameter('max_cluster_points').value)
        if self._odom_timeout_s <= 0 or self._max_cluster_points < 1:
            raise ValueError('odom_timeout_s and max_cluster_points must be positive')
        self._dedup = SpatialDeduplicator(
            cell_m=float(self.get_parameter('dedup_cell_m').value),
            ttl_s=float(self.get_parameter('dedup_ttl_s').value),
        )

        lidar = str(self.get_parameter('lidar_topic').value)
        self._rocks_cloud_topic = str(self.get_parameter('rocks_cloud_topic').value)
        self.pub_rocks = self.create_publisher(
            PointCloud2, self._rocks_cloud_topic, qos_profile_sensor_data
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
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._on_odom,
            10,
        )
        self.create_timer(2.0, self._heartbeat)
        self.get_logger().info(f'edge_rock_detect listening on {lidar}')

    def _on_odom(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._pose = Pose2D(float(position.x), float(position.y), float(position.z), float(yaw))
        self._pose_received_at = time.monotonic()

    def _on_lidar(self, msg: PointCloud2) -> None:
        try:
            self._process_lidar(msg)
        except (ValueError, TypeError, IndexError) as exc:
            self._bad_frames += 1
            self.get_logger().warn(f'ignoring malformed lidar frame: {exc}')

    def _process_lidar(self, msg: PointCloud2) -> None:
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
        if obs.shape[0] > self._max_cluster_points:
            keep = np.linspace(0, obs.shape[0] - 1, self._max_cluster_points).astype(int)
            obs = obs[keep]
            hag_obs = hag_obs[keep]

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

        sensor_t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        ros_now = float(self.get_clock().now().nanoseconds) * 1e-9
        t_ros = sensor_t if sensor_t > 0.0 else ros_now
        input_frame = msg.header.frame_id or 'base_link'
        for detection in dets:
            detection.frame_id = input_frame
        dset = DetectionSet(t=sensor_t, frame_id=input_frame, source_node='edge_rock_detect', detections=dets)
        s = String()
        s.data = json.dumps(dset.to_dict())
        self.pub_det.publish(s)

        monotonic_now = time.monotonic()
        pose_fresh = (
            self._pose is not None
            and monotonic_now - self._pose_received_at <= self._odom_timeout_s
        )
        for d in dets:
            if d.confidence < self.alert_min_conf:
                continue
            if pose_fresh and self._pose is not None:
                alert_x, alert_y, alert_z = body_to_map(d.x, d.y, d.z, self._pose)
                alert_frame = self.frame_map
            else:
                alert_x, alert_y, alert_z = d.x, d.y, d.z
                alert_frame = input_frame
            if self._dedup.seen_recently(alert_x, alert_y, monotonic_now):
                continue
            sev = Severity.WARN if d.radius_m >= 0.7 or abs(d.y) < 3.0 else Severity.INFO
            if d.details.get('range_m', 99) < 10 and abs(d.y) < 2.5:
                sev = Severity.CRITICAL
            alert = AlertEvent(
                type='rock',
                severity=sev,
                confidence=d.confidence,
                source_node='edge_rock_detect',
                vehicle_id=self.vehicle_id,
                frame_id=alert_frame,
                t_ros=t_ros,
                t_vehicle=sensor_t,
                x=alert_x,
                y=alert_y,
                z=alert_z,
                geometry={'kind': 'sphere', 'radius_m': d.radius_m, 'extent_m': list(d.extent_m)},
                details={**d.details, 'pose_valid': pose_fresh},
                cloud_ref={'topic': self._rocks_cloud_topic, 'stamp': sensor_t},
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
                'tracks_cached': len(self._dedup),
                'bad_frames': self._bad_frames,
                'odom_fresh': time.monotonic() - self._pose_received_at <= self._odom_timeout_s,
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
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
