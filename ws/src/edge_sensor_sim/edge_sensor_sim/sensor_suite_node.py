#!/usr/bin/env python3
"""ROS 2 node: subscribe to StreamHub (sole generator) and publish standard messages."""

from __future__ import annotations

import os
import struct
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from edge_sensor_sim.bus.contract import SensorBusContract
from edge_sensor_sim.maps import DEFAULT_PLAYLIST
from edge_sensor_sim.stream.format import default_streams_root
from edge_sensor_sim.stream.hub import StreamConfig, StreamHub


def points_to_cloud(
    points: np.ndarray,
    intensity: np.ndarray,
    header: Header,
) -> PointCloud2:
    """Pack xyz + intensity float32 PointCloud2."""
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
    msg.row_step = msg.point_step * n
    buf = bytearray(msg.row_step)
    for i in range(n):
        struct.pack_into(
            '<ffff',
            buf,
            i * 16,
            float(points[i, 0]),
            float(points[i, 1]),
            float(points[i, 2]),
            float(intensity[i]) if i < len(intensity) else 0.0,
        )
    msg.data = bytes(buf)
    return msg


def _lidar_arrays(lidar: Dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    xy = np.asarray(lidar.get('xy') or [], dtype=np.float32)
    z = np.asarray(lidar.get('z') or [], dtype=np.float32)
    inten = np.asarray(lidar.get('i') or [], dtype=np.float32)
    if xy.size == 0:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0,), dtype=np.float32)
    if xy.ndim == 1:
        xy = xy.reshape(-1, 2)
    n = xy.shape[0]
    pts = np.zeros((n, 3), dtype=np.float32)
    pts[:, :2] = xy[:n]
    if z.size:
        pts[:, 2] = z[:n]
    if inten.size < n:
        inten = np.resize(inten, n)
    return pts, inten.astype(np.float32)


class SensorSuiteNode(Node):
    """
    ROS bridge over StreamHub.

    The hub is the sole sensor generator (live multi-map or replay).
    This node only subscribes to hub state and republishes ROS messages.
    """

    def __init__(self, hub: Optional[StreamHub] = None) -> None:
        super().__init__('sensor_suite')

        self.declare_parameter('imu_hz', 100.0)
        self.declare_parameter('gnss_hz', 5.0)
        self.declare_parameter('lidar_hz', 10.0)
        self.declare_parameter('odom_hz', 50.0)
        self.declare_parameter('speed_mps', 8.0)
        self.declare_parameter('duration_s', float(os.environ.get('DURATION', '60')))
        self.declare_parameter('stream_mode', os.environ.get('STREAM_MODE', 'live'))
        self.declare_parameter('maps', os.environ.get('MAPS', ','.join(DEFAULT_PLAYLIST)))
        self.declare_parameter('stream_path', os.environ.get('STREAM_PATH', ''))
        self.declare_parameter('streams_root', os.environ.get('STREAMS_ROOT', ''))
        self.declare_parameter('cycle_maps', True)
        bus = SensorBusContract()
        self.declare_parameter('imu_topic', bus.imu_topic)
        self.declare_parameter('gnss_topic', bus.gnss_topic)
        self.declare_parameter('lidar_topic', bus.lidar_topic)
        self.declare_parameter('odom_topic', bus.odom_topic)
        self.declare_parameter('sim_odom_topic', bus.sim_odom_topic)
        self.declare_parameter('frame_base', bus.frame_base)
        self.declare_parameter('frame_imu', bus.frame_imu)
        self.declare_parameter('frame_lidar', bus.frame_lidar)
        self.declare_parameter('frame_map', bus.frame_map)
        self.declare_parameter('publish_tf', True)

        imu_hz = float(self.get_parameter('imu_hz').value)
        gnss_hz = float(self.get_parameter('gnss_hz').value)
        lidar_hz = float(self.get_parameter('lidar_hz').value)
        odom_hz = float(self.get_parameter('odom_hz').value)
        speed = float(self.get_parameter('speed_mps').value)
        duration_s = float(self.get_parameter('duration_s').value)
        mode = str(self.get_parameter('stream_mode').value or 'live')
        maps_raw = str(self.get_parameter('maps').value or '')
        map_ids = [m.strip() for m in maps_raw.split(',') if m.strip()] or list(DEFAULT_PLAYLIST)
        stream_path = str(self.get_parameter('stream_path').value or '') or None
        streams_root = str(self.get_parameter('streams_root').value or '') or str(default_streams_root())
        cycle_maps = bool(self.get_parameter('cycle_maps').value)

        self.frame_base = str(self.get_parameter('frame_base').value)
        self.frame_imu = str(self.get_parameter('frame_imu').value)
        self.frame_lidar = str(self.get_parameter('frame_lidar').value)
        self.frame_map = str(self.get_parameter('frame_map').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        if hub is None:
            if stream_path:
                mode = 'replay'
            cfg = StreamConfig(
                mode=mode,
                map_ids=map_ids,
                stream_path=stream_path,
                streams_root=streams_root,
                duration_s=duration_s,
                loop=True,
                cycle_maps=cycle_maps,
                imu_hz=min(imu_hz, 50.0),  # hub internal rates; ROS pub rates separate
                gnss_hz=gnss_hz,
                lidar_hz=min(lidar_hz, 10.0),
                vehicle_hz=odom_hz,
                history_s=duration_s,
                speed_mps=speed,
            )
            self.hub = StreamHub(cfg)
            self._owns_hub = True
        else:
            self.hub = hub
            self._owns_hub = False

        imu_topic = str(self.get_parameter('imu_topic').value)
        gnss_topic = str(self.get_parameter('gnss_topic').value)
        lidar_topic = str(self.get_parameter('lidar_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        sim_odom_topic = str(self.get_parameter('sim_odom_topic').value)

        # Publish onto the canonical sensor bus (same topics real drivers should use).
        self.pub_imu = self.create_publisher(Imu, imu_topic, qos_profile_sensor_data)
        self.pub_gnss = self.create_publisher(NavSatFix, gnss_topic, qos_profile_sensor_data)
        self.pub_lidar = self.create_publisher(PointCloud2, lidar_topic, qos_profile_sensor_data)
        self.pub_odom = self.create_publisher(Odometry, odom_topic, 10)
        self.pub_sim_odom = None
        if sim_odom_topic and sim_odom_topic != odom_topic:
            self.pub_sim_odom = self.create_publisher(Odometry, sim_odom_topic, 10)

        self._tf_broadcaster: Optional[TransformBroadcaster] = None
        self._static_tf: Optional[StaticTransformBroadcaster] = None
        if self.publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self)
            self._static_tf = StaticTransformBroadcaster(self)
            self._publish_static_tf()

        self._last_imu_t = -1.0
        self._last_gnss_t = -1.0
        self._last_lidar_t = -1.0
        self._last_odom_t = -1.0

        if self._owns_hub:
            self.hub.start()

        # Poll hub at the highest publish rate; emit only on new sample timestamps.
        poll_hz = max(imu_hz, odom_hz, lidar_hz, gnss_hz, 20.0)
        self.create_timer(1.0 / poll_hz, self._on_poll)

        cat = self.hub.catalog()
        self.get_logger().info(
            f'sensor_suite subscribed to StreamHub mode={self.hub.cfg.mode} '
            f'source={self.hub.snapshot().get("source")} maps={[m["id"] for m in cat["maps"]]}'
        )

    def destroy_node(self) -> bool:
        if self._owns_hub:
            self.hub.stop()
        return super().destroy_node()

    def _header(self, frame_id: str) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame_id
        return h

    def _publish_static_tf(self) -> None:
        assert self._static_tf is not None
        statics = []
        for child, z in ((self.frame_imu, 0.0), (self.frame_lidar, 2.5)):
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = self.frame_base
            tf.child_frame_id = child
            tf.transform.translation.x = 0.0
            tf.transform.translation.y = 0.0
            tf.transform.translation.z = float(z)
            tf.transform.rotation.w = 1.0
            statics.append(tf)
        self._static_tf.sendTransform(statics)

    def _on_poll(self) -> None:
        snap = self.hub.snapshot()
        imu = snap['latest'].get('imu')
        gnss = snap['latest'].get('gnss')
        odom = snap['latest'].get('odom')
        lidar = snap.get('lidar')

        if imu and float(imu.get('t', -1.0)) > self._last_imu_t:
            self._last_imu_t = float(imu['t'])
            self._publish_imu(imu)

        if gnss and float(gnss.get('t', -1.0)) > self._last_gnss_t:
            self._last_gnss_t = float(gnss['t'])
            self._publish_gnss(gnss)

        if odom and float(odom.get('t', -1.0)) > self._last_odom_t:
            self._last_odom_t = float(odom['t'])
            self._publish_odom(odom)

        if lidar and float(lidar.get('t', -1.0)) > self._last_lidar_t:
            self._last_lidar_t = float(lidar['t'])
            self._publish_lidar(lidar)

    def _publish_imu(self, s: Dict[str, Any]) -> None:
        msg = Imu()
        msg.header = self._header(self.frame_imu)
        msg.angular_velocity.x = float(s['gx'])
        msg.angular_velocity.y = float(s['gy'])
        msg.angular_velocity.z = float(s['gz'])
        msg.linear_acceleration.x = float(s['ax'])
        msg.linear_acceleration.y = float(s['ay'])
        msg.linear_acceleration.z = float(s['az'])
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0
        self.pub_imu.publish(msg)

    def _publish_gnss(self, s: Dict[str, Any]) -> None:
        msg = NavSatFix()
        msg.header = self._header(self.frame_map)
        msg.latitude = float(s['lat'])
        msg.longitude = float(s['lon'])
        msg.altitude = float(s['alt'])
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        msg.position_covariance[0] = 2.25
        msg.position_covariance[4] = 2.25
        msg.position_covariance[8] = 9.0
        fix_ok = bool(s.get('fix_ok', True))
        msg.status.status = (
            NavSatStatus.STATUS_SBAS_FIX if fix_ok else NavSatStatus.STATUS_NO_FIX
        )
        msg.status.service = NavSatStatus.SERVICE_GPS
        self.pub_gnss.publish(msg)

    def _publish_lidar(self, fr: Dict[str, Any]) -> None:
        pts, inten = _lidar_arrays(fr)
        header = self._header(self.frame_lidar)
        cloud = points_to_cloud(pts, inten, header)
        self.pub_lidar.publish(cloud)

    def _publish_odom(self, st: Dict[str, Any]) -> None:
        msg = Odometry()
        msg.header = self._header(self.frame_map)
        msg.child_frame_id = self.frame_base
        msg.pose.pose.position.x = float(st['x'])
        msg.pose.pose.position.y = float(st['y'])
        msg.pose.pose.position.z = float(st['z'])
        half = 0.5 * float(st['yaw'])
        msg.pose.pose.orientation.z = float(np.sin(half))
        msg.pose.pose.orientation.w = float(np.cos(half))
        msg.twist.twist.linear.x = float(st.get('vx', 0.0))
        msg.twist.twist.linear.y = float(st.get('vy', 0.0))
        msg.twist.twist.linear.z = float(st.get('vz', 0.0))
        msg.twist.twist.angular.z = float(st.get('yaw_rate', 0.0))
        self.pub_odom.publish(msg)
        if self.pub_sim_odom is not None:
            self.pub_sim_odom.publish(msg)

        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = msg.header
            tf.child_frame_id = self.frame_base
            tf.transform.translation.x = msg.pose.pose.position.x
            tf.transform.translation.y = msg.pose.pose.position.y
            tf.transform.translation.z = msg.pose.pose.position.z
            tf.transform.rotation = msg.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SensorSuiteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
