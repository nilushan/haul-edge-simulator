#!/usr/bin/env python3
"""ROS 2 node: sample pure-Python sensor models and publish standard messages."""

from __future__ import annotations

import struct
from typing import Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from edge_sensor_sim.models import (
    GnssSimulator,
    HaulWorld,
    ImuSimulator,
    LidarSimulator,
    VehicleSimulator,
)


def _stamp(node: Node, t_sec: float) -> Time:
    # Use simulation time offset from node start wall clock for simplicity
    msg = Time()
    msg.sec = int(t_sec)
    msg.nanosec = int((t_sec - int(t_sec)) * 1e9)
    return msg


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


class SensorSuiteNode(Node):
    def __init__(self) -> None:
        super().__init__('sensor_suite')

        self.declare_parameter('imu_hz', 100.0)
        self.declare_parameter('gnss_hz', 5.0)
        self.declare_parameter('lidar_hz', 10.0)
        self.declare_parameter('odom_hz', 50.0)
        self.declare_parameter('speed_mps', 8.0)
        self.declare_parameter('frame_base', 'base_link')
        self.declare_parameter('frame_imu', 'imu_link')
        self.declare_parameter('frame_lidar', 'lidar_link')
        self.declare_parameter('frame_map', 'map')
        self.declare_parameter('publish_tf', True)

        imu_hz = float(self.get_parameter('imu_hz').value)
        gnss_hz = float(self.get_parameter('gnss_hz').value)
        lidar_hz = float(self.get_parameter('lidar_hz').value)
        odom_hz = float(self.get_parameter('odom_hz').value)
        speed = float(self.get_parameter('speed_mps').value)

        self.frame_base = str(self.get_parameter('frame_base').value)
        self.frame_imu = str(self.get_parameter('frame_imu').value)
        self.frame_lidar = str(self.get_parameter('frame_lidar').value)
        self.frame_map = str(self.get_parameter('frame_map').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        world = HaulWorld(seed=19)
        self.vehicle = VehicleSimulator(speed_mps=speed, world=world)
        self.imu_model = ImuSimulator()
        self.gnss_model = GnssSimulator()
        self.lidar_model = LidarSimulator(world=world)

        self.pub_imu = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.pub_gnss = self.create_publisher(NavSatFix, '/gnss/fix', qos_profile_sensor_data)
        self.pub_lidar = self.create_publisher(PointCloud2, '/lidar', qos_profile_sensor_data)
        self.pub_odom = self.create_publisher(Odometry, '/sim/ground_truth/odom', 10)

        self._tf_broadcaster: Optional[TransformBroadcaster] = None
        self._static_tf: Optional[StaticTransformBroadcaster] = None
        if self.publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self)
            self._static_tf = StaticTransformBroadcaster(self)
            self._publish_static_tf()

        self._t0 = self.get_clock().now()
        self.vehicle.reset(0.0)
        self._last_st = self.vehicle.step(0.0)

        self.create_timer(1.0 / imu_hz, self._on_imu)
        self.create_timer(1.0 / gnss_hz, self._on_gnss)
        self.create_timer(1.0 / lidar_hz, self._on_lidar)
        self.create_timer(1.0 / odom_hz, self._on_odom)

        self.get_logger().info(
            f'sensor_suite started  imu={imu_hz}Hz gnss={gnss_hz}Hz lidar={lidar_hz}Hz speed={speed}m/s'
        )

    def _sim_time(self) -> float:
        dt = self.get_clock().now() - self._t0
        return dt.nanoseconds * 1e-9

    def _header(self, frame_id: str, t: float) -> Header:
        h = Header()
        h.stamp = _stamp(self, t)
        # Prefer ROS clock stamp for tooling that ignores custom sec
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame_id
        return h

    def _advance(self):
        t = self._sim_time()
        self._last_st = self.vehicle.step(t)
        return self._last_st

    def _publish_static_tf(self) -> None:
        assert self._static_tf is not None
        statics = []
        # imu at origin of base
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

    def _on_imu(self) -> None:
        st = self._advance()
        s = self.imu_model.sample(st)
        msg = Imu()
        msg.header = self._header(self.frame_imu, s.t)
        msg.angular_velocity.x = s.gx
        msg.angular_velocity.y = s.gy
        msg.angular_velocity.z = s.gz
        msg.linear_acceleration.x = s.ax
        msg.linear_acceleration.y = s.ay
        msg.linear_acceleration.z = s.az
        # orientation unknown (no mag fusion) — leave identity + high cov
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0
        self.pub_imu.publish(msg)

    def _on_gnss(self) -> None:
        st = self._advance()
        s = self.gnss_model.sample(st)
        msg = NavSatFix()
        msg.header = self._header(self.frame_map, s.t)
        msg.latitude = s.latitude_deg
        msg.longitude = s.longitude_deg
        msg.altitude = s.altitude_m
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        msg.position_covariance[0] = s.h_acc_m ** 2
        msg.position_covariance[4] = s.h_acc_m ** 2
        msg.position_covariance[8] = s.v_acc_m ** 2
        msg.status.status = (
            NavSatStatus.STATUS_SBAS_FIX if s.fix_ok else NavSatStatus.STATUS_NO_FIX
        )
        msg.status.service = NavSatStatus.SERVICE_GPS
        self.pub_gnss.publish(msg)

    def _on_lidar(self) -> None:
        st = self._advance()
        fr = self.lidar_model.sample(self.vehicle, st)
        header = self._header(self.frame_lidar, fr.t)
        cloud = points_to_cloud(fr.points, fr.intensity, header)
        self.pub_lidar.publish(cloud)

    def _on_odom(self) -> None:
        st = self._advance()
        msg = Odometry()
        msg.header = self._header(self.frame_map, st.t)
        msg.child_frame_id = self.frame_base
        msg.pose.pose.position.x = st.x
        msg.pose.pose.position.y = st.y
        msg.pose.pose.position.z = st.z
        # yaw-only quaternion
        half = 0.5 * st.yaw
        msg.pose.pose.orientation.z = float(np.sin(half))
        msg.pose.pose.orientation.w = float(np.cos(half))
        msg.twist.twist.linear.x = st.vx
        msg.twist.twist.linear.y = st.vy
        msg.twist.twist.linear.z = st.vz
        msg.twist.twist.angular.z = st.yaw_rate
        self.pub_odom.publish(msg)

        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = msg.header
            tf.child_frame_id = self.frame_base
            tf.transform.translation.x = st.x
            tf.transform.translation.y = st.y
            tf.transform.translation.z = st.z
            tf.transform.rotation = msg.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf)


def main(args=None) -> None:
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
