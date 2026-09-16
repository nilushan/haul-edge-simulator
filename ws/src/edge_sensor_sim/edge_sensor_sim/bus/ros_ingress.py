"""ROS 2 ingress: subscribe to the sensor bus and fill a TickBuffer for viz/processors."""

from __future__ import annotations

import math
import struct
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from edge_sensor_sim.bus.contract import SensorBusContract, default_bus_contract
from edge_sensor_sim.bus.tick_buffer import TickBuffer, TickBufferConfig


def _stamp_to_t(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _cloud_to_lidar_dict(msg: Any, max_points: int = 4000) -> Dict[str, Any]:
    """Decode PointCloud2 xyz[+intensity] into viz lidar payload."""
    # Field offsets
    offsets = {f.name: f.offset for f in msg.fields}
    if not {'x', 'y', 'z'}.issubset(offsets):
        return {'t': _stamp_to_t(msg.header.stamp), 'n': 0, 'xy': [], 'z': [], 'i': []}

    step = int(msg.point_step)
    n = int(msg.width) * int(msg.height)
    data = memoryview(msg.data)
    xs: List[float] = []
    ys: List[float] = []
    zs: List[float] = []
    inten: List[float] = []
    has_i = 'intensity' in offsets
    ox, oy, oz = offsets['x'], offsets['y'], offsets['z']
    oi = offsets.get('intensity', 0)

    # stride sample if huge
    stride = max(1, n // max_points) if n > max_points else 1
    for i in range(0, n, stride):
        base = i * step
        if base + 12 > len(data):
            break
        x = struct.unpack_from('<f', data, base + ox)[0]
        y = struct.unpack_from('<f', data, base + oy)[0]
        z = struct.unpack_from('<f', data, base + oz)[0]
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        xs.append(float(x))
        ys.append(float(y))
        zs.append(float(z))
        if has_i and base + oi + 4 <= len(data):
            inten.append(float(struct.unpack_from('<f', data, base + oi)[0]))
        else:
            inten.append(1.0)
        if len(xs) >= max_points:
            break

    xy = [[round(xs[i], 3), round(ys[i], 3)] for i in range(len(xs))]
    return {
        't': _stamp_to_t(msg.header.stamp),
        'n': len(xs),
        'xy': xy,
        'z': [round(z, 3) for z in zs],
        'i': [round(v, 3) for v in inten],
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
        max_lidar_points: int = 4000,
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
        # Optional JSON status from processors
        try:
            node.create_subscription(String, c.edge_status_topic, self._on_status, 10)
        except Exception:  # noqa: BLE001
            pass

        buffer.set_source_meta(source=f'bus:{c.imu_topic}', mode='bus')
        node.get_logger().info(
            f'RosBusIngress listening  imu={c.imu_topic} gnss={c.gnss_topic} '
            f'lidar={c.lidar_topic} odom={c.odom_topic}'
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

    def _on_status(self, msg: Any) -> None:
        try:
            data = self._json.loads(msg.data)
        except Exception:  # noqa: BLE001
            data = {'raw': getattr(msg, 'data', '')}
        self.buffer.push_edge_status(data)
