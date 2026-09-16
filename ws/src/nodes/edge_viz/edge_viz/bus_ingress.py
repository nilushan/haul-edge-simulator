"""ROS 2 ingress: subscribe to the sensor bus and fill a TickBuffer for viz/processors."""

from __future__ import annotations

import math
import struct
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from edge_perception.cloud_io import decode_xyz
from edge_sim.topics import SensorBusContract, default_bus_contract
from edge_viz.tick_buffer import TickBuffer


def _stamp_to_t(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _decode_optional_float32(msg: Any, name: str, count: int, default: float) -> np.ndarray:
    field = next((f for f in msg.fields if f.name == name), None)
    if field is None:
        return np.full((count,), default, dtype=np.float32)
    if int(getattr(field, 'datatype', 7)) != 7 or int(getattr(field, 'count', 1)) != 1:
        return np.full((count,), default, dtype=np.float32)
    step = int(msg.point_step)
    offset = int(field.offset)
    if step <= 0 or offset < 0 or offset + 4 > step:
        return np.full((count,), default, dtype=np.float32)
    width, height = int(msg.width), int(msg.height)
    row_step = int(getattr(msg, 'row_step', 0)) or width * step
    data = memoryview(msg.data)
    fmt = '>f' if bool(getattr(msg, 'is_bigendian', False)) else '<f'
    values = np.full((count,), default, dtype=np.float32)
    index = 0
    for row in range(max(height, 0)):
        for col in range(max(width, 0)):
            if index >= count:
                return values
            base = row * row_step + col * step
            if base + step > len(data):
                return values
            values[index] = struct.unpack_from(fmt, data, base + offset)[0]
            index += 1
    return values


def _cloud_to_lidar_dict(msg: Any, max_points: int = 4000) -> Dict[str, Any]:
    """Decode a validated PointCloud2 into the compact browser payload."""
    if max_points < 1:
        raise ValueError('max_points must be positive')
    points, intensity = decode_xyz(msg)
    count = int(points.shape[0])
    labels = _decode_optional_float32(msg, 'label', count, -1.0)
    confidence = _decode_optional_float32(msg, 'conf', count, 1.0)
    if intensity is None:
        intensity = np.ones((count,), dtype=np.float32)

    finite = np.all(np.isfinite(points), axis=1)
    points = points[finite]
    intensity = intensity[finite]
    labels = labels[finite]
    confidence = confidence[finite]
    if points.shape[0] > max_points:
        indices = np.linspace(0, points.shape[0] - 1, max_points).astype(int)
        points = points[indices]
        intensity = intensity[indices]
        labels = labels[indices]
        confidence = confidence[indices]

    return {
        't': _stamp_to_t(msg.header.stamp),
        'n': int(points.shape[0]),
        'xy': np.round(points[:, :2], 3).tolist(),
        'z': np.round(points[:, 2], 3).tolist(),
        'i': np.round(intensity.astype(float), 3).tolist(),
        'label': np.round(labels.astype(float), 3).tolist(),
        'conf': np.round(confidence.astype(float), 3).tolist(),
    }


def imu_msg_to_dict(msg: Any) -> Dict[str, Any]:
    return {
        't': _stamp_to_t(msg.header.stamp),
        'gx': float(msg.angular_velocity.x),
        'gy': float(msg.angular_velocity.y),
        'gz': float(msg.angular_velocity.z),
        'ax': float(msg.linear_acceleration.x),
        'ay': float(msg.linear_acceleration.y),
        'az': float(msg.linear_acceleration.z),
    }


def gnss_msg_to_dict(msg: Any) -> Dict[str, Any]:
    # NavSatStatus: STATUS_NO_FIX = -1 in some distros; treat >= 0 as ok
    status = int(getattr(msg.status, 'status', 0))
    return {
        't': _stamp_to_t(msg.header.stamp),
        'lat': float(msg.latitude),
        'lon': float(msg.longitude),
        'alt': float(msg.altitude),
        'fix_ok': status >= 0,
        'status': status,
    }


def odom_msg_to_dict(msg: Any) -> Dict[str, Any]:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    # yaw from quaternion (z,w dominant for planar)
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    tw = msg.twist.twist
    vx, vy, vz = float(tw.linear.x), float(tw.linear.y), float(tw.linear.z)
    return {
        't': _stamp_to_t(msg.header.stamp),
        'x': float(p.x),
        'y': float(p.y),
        'z': float(p.z),
        'yaw': float(yaw),
        'pitch': 0.0,
        'roll': 0.0,
        'vx': vx,
        'vy': vy,
        'vz': vz,
        'yaw_rate': float(tw.angular.z),
        'pitch_rate': 0.0,
        'roll_rate': 0.0,
        'ax': 0.0,
        'ay': 0.0,
        'az': 0.0,
        'speed': float(math.hypot(vx, vy)),
    }


class RosBusIngress:
    """
    rclpy node helper: subscribe to SensorBusContract topics → TickBuffer.

    Used by viz_bus_server so the browser sees *whatever* is on the bus
    (sim publisher, bag play, or real drivers) with no code change.
    """

    def __init__(
        self,
        node: Any,
        buffer: TickBuffer,
        contract: Optional[SensorBusContract] = None,
        *,
        max_lidar_points: int = 20000,
        also_sim_odom: bool = True,
    ) -> None:
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        import json

        self.node = node
        self.buffer = buffer
        self.contract = contract or default_bus_contract()
        self.max_lidar_points = max_lidar_points
        self._json = json

        c = self.contract
        sensor_qos = qos_profile_sensor_data

        node.create_subscription(Imu, c.imu_topic, self._on_imu, sensor_qos)
        node.create_subscription(NavSatFix, c.gnss_topic, self._on_gnss, sensor_qos)
        node.create_subscription(PointCloud2, c.lidar_topic, self._on_lidar, sensor_qos)
        node.create_subscription(Odometry, c.odom_topic, self._on_odom, 10)
        if also_sim_odom and c.sim_odom_topic != c.odom_topic:
            node.create_subscription(Odometry, c.sim_odom_topic, self._on_odom, 10)
        node.create_subscription(
            PointCloud2, c.processed_lidar_topic, self._on_processed_lidar, sensor_qos
        )
        # Detection layers + alert stream
        node.create_subscription(PointCloud2, c.ground_cloud_topic, self._on_ground, sensor_qos)
        node.create_subscription(PointCloud2, c.obstacles_cloud_topic, self._on_obstacles, sensor_qos)
        node.create_subscription(PointCloud2, c.rocks_cloud_topic, self._on_rocks, sensor_qos)
        node.create_subscription(PointCloud2, c.bunds_cloud_topic, self._on_bunds, sensor_qos)
        node.create_subscription(String, c.detections_topic, self._on_detections, 20)
        node.create_subscription(String, c.alerts_topic, self._on_alert, 50)
        node.create_subscription(String, c.vibe_features_topic, self._on_vibe_features, 10)
        # Optional JSON status from processors
        try:
            node.create_subscription(String, c.edge_status_topic, self._on_status, 10)
        except Exception:  # noqa: BLE001
            pass

        buffer.set_source_meta(source=f'bus:{c.imu_topic}', mode='bus')
        node.get_logger().info(
            f'RosBusIngress listening  imu={c.imu_topic} gnss={c.gnss_topic} '
            f'lidar={c.lidar_topic} odom={c.odom_topic} '
            f'rocks={c.rocks_cloud_topic} alerts={c.alerts_topic}'
        )

    def _on_imu(self, msg: Any) -> None:
        self.buffer.push_imu(imu_msg_to_dict(msg))

    def _on_gnss(self, msg: Any) -> None:
        self.buffer.push_gnss(gnss_msg_to_dict(msg))

    def _on_odom(self, msg: Any) -> None:
        self.buffer.push_odom(odom_msg_to_dict(msg))

    def _on_lidar(self, msg: Any) -> None:
        self.buffer.push_lidar(_cloud_to_lidar_dict(msg, self.max_lidar_points), processed=False)

    def _on_processed_lidar(self, msg: Any) -> None:
        self.buffer.push_lidar(_cloud_to_lidar_dict(msg, self.max_lidar_points), processed=True)

    def _on_ground(self, msg: Any) -> None:
        self.buffer.push_detect_cloud('ground', _cloud_to_lidar_dict(msg, self.max_lidar_points))

    def _on_obstacles(self, msg: Any) -> None:
        self.buffer.push_detect_cloud('obstacles', _cloud_to_lidar_dict(msg, self.max_lidar_points))

    def _on_rocks(self, msg: Any) -> None:
        self.buffer.push_detect_cloud('rocks', _cloud_to_lidar_dict(msg, self.max_lidar_points))

    def _on_bunds(self, msg: Any) -> None:
        self.buffer.push_detect_cloud('bunds', _cloud_to_lidar_dict(msg, self.max_lidar_points))

    def _on_detections(self, msg: Any) -> None:
        try:
            data = self._json.loads(msg.data)
        except Exception:  # noqa: BLE001
            return
        if isinstance(data, dict):
            self.buffer.push_detections(data)

    def _on_alert(self, msg: Any) -> None:
        try:
            data = self._json.loads(msg.data)
        except Exception:  # noqa: BLE001
            return
        if isinstance(data, dict):
            self.buffer.push_alert(data)

    def _on_vibe_features(self, msg: Any) -> None:
        try:
            data = self._json.loads(msg.data)
        except Exception:  # noqa: BLE001
            return
        if isinstance(data, dict):
            self.buffer.push_vibe_features(data)

    def _on_status(self, msg: Any) -> None:
        try:
            data = self._json.loads(msg.data)
        except Exception:  # noqa: BLE001
            data = {'raw': getattr(msg, 'data', '')}
        self.buffer.push_edge_status(data if isinstance(data, dict) else {'raw': str(data)})
