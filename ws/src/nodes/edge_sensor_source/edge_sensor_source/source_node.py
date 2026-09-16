#!/usr/bin/env python3
"""ROS 2 node: subscribe to StreamHub (sole generator) and publish standard messages."""

from __future__ import annotations

import os
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

from edge_sim.topics import SensorBusContract
from edge_sim.maps import DEFAULT_PLAYLIST
from edge_sim.stream.format import default_streams_root
from edge_sim.stream import StreamConfig, StreamHub
from edge_sensor_source.conversion import lidar_arrays as _lidar_arrays, xyzi_bytes


def points_to_cloud(
    points: np.ndarray,
    intensity: np.ndarray,
    header: Header,
) -> PointCloud2:
    """Pack aligned xyz + intensity arrays into a PointCloud2."""
    points = np.asarray(points, dtype=np.float32)
    intensity = np.asarray(intensity, dtype=np.float32)
    data = xyzi_bytes(points, intensity)
    n = int(points.shape[0])
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.is_bigendian = False
    msg.is_dense = bool(np.all(np.isfinite(points)) and np.all(np.isfinite(intensity)))
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16
    msg.row_step = msg.point_step * n
    msg.data = data
    return msg


class SensorSourceNode(Node):
    """
    ROS bridge over StreamHub.

    The hub is the sole sensor generator (live multi-map or replay).
    This node only subscribes to hub state and republishes ROS messages.
    """

    def __init__(self, hub: Optional[StreamHub] = None) -> None:
        super().__init__('sensor_source')

        self.declare_parameter('imu_hz', 100.0)
        self.declare_parameter('gnss_hz', 5.0)
        self.declare_parameter('lidar_hz', 10.0)
        self.declare_parameter('odom_hz', 50.0)
        self.declare_parameter('speed_mps', 8.0)
        self.declare_parameter('duration_s', float(os.environ.get('DURATION', '600')))
        self.declare_parameter('stream_mode', os.environ.get('STREAM_MODE', 'live'))
        self.declare_parameter('maps', os.environ.get('MAPS', ','.join(DEFAULT_PLAYLIST)))
        self.declare_parameter('stream_path', os.environ.get('STREAM_PATH', ''))
        self.declare_parameter('streams_root', os.environ.get('STREAMS_ROOT', ''))
        self.declare_parameter('cycle_maps', False)
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
                imu_hz=imu_hz,
                gnss_hz=gnss_hz,
                lidar_hz=lidar_hz,
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
        self._bad_samples = 0

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
            try:
                self.hub.stop()
            except RuntimeError as exc:
                self.get_logger().error(f'failed to stop StreamHub cleanly: {exc}')
        return super().destroy_node()

    def _header(self, frame_id: str) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame_id
        return h

    def _publish_static_tf(self) -> None:
        assert self._static_tf is not None
        statics = []
        for child, xyz in (
            (self.frame_imu, (0.0, 0.0, 0.0)),
            (self.frame_lidar, (2.5, 0.0, 3.2)),
        ):
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = self.frame_base
            tf.child_frame_id = child
            tf.transform.translation.x = float(xyz[0])
            tf.transform.translation.y = float(xyz[1])
            tf.transform.translation.z = float(xyz[2])
            tf.transform.rotation.w = 1.0
            statics.append(tf)
        self._static_tf.sendTransform(statics)

    def _publish_new_sample(
        self,
        kind: str,
        sample: Optional[Dict[str, Any]],
        last_attr: str,
        publish: Any,
    ) -> None:
        if not sample:
            return
        try:
            timestamp = float(sample.get('t', -1.0))
        except (TypeError, ValueError, AttributeError):
            self._bad_samples += 1
            self.get_logger().warn(f'ignoring malformed {kind} sample timestamp')
            return
        if not np.isfinite(timestamp) or timestamp <= float(getattr(self, last_attr)):
            return
        try:
            publish(sample)
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            self._bad_samples += 1
            setattr(self, last_attr, timestamp)
            self.get_logger().warn(f'ignoring malformed {kind} sample: {exc}')
            return
        setattr(self, last_attr, timestamp)

    def _on_poll(self) -> None:
        snap = self.hub.snapshot()
        latest = snap.get('latest') or {}
        self._publish_new_sample('imu', latest.get('imu'), '_last_imu_t', self._publish_imu)
        self._publish_new_sample('gnss', latest.get('gnss'), '_last_gnss_t', self._publish_gnss)
        self._publish_new_sample('odom', latest.get('odom'), '_last_odom_t', self._publish_odom)
        self._publish_new_sample('lidar', snap.get('lidar'), '_last_lidar_t', self._publish_lidar)

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
        h_acc = max(0.0, float(s.get('h_acc_m', 1.5)))
        v_acc = max(0.0, float(s.get('v_acc_m', 3.0)))
        msg.position_covariance[0] = h_acc * h_acc
        msg.position_covariance[4] = h_acc * h_acc
        msg.position_covariance[8] = v_acc * v_acc
        fix_ok = bool(s.get('fix_ok', True))
        msg.status.status = (
            NavSatStatus.STATUS_SBAS_FIX if fix_ok else NavSatStatus.STATUS_NO_FIX
        )
        msg.status.service = NavSatStatus.SERVICE_GPS
        self.pub_gnss.publish(msg)

    def _publish_lidar(self, fr: Dict[str, Any]) -> None:
        pts, inten = _lidar_arrays(fr)
        # LidarSimulator returns points in base_link coordinates; publish that
        # frame while retaining the accurate base->lidar static mount TF.
        header = self._header(self.frame_base)
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
    node = SensorSourceNode()
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
