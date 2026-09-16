#!/usr/bin/env python3
"""IMU vibration detector: /imu/data + /odom → /edge/alerts."""

from __future__ import annotations

import json
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from edge_perception.vibration import ImuSample, VibrationMonitor, VibeParams
from edge_sim.topics import SensorBusContract
from edge_vibe_detect.health import is_fresh, validate_thresholds


class VibeDetectNode(Node):
    def __init__(self) -> None:
        super().__init__('edge_vibe_detect')
        c = SensorBusContract()
        self.declare_parameter('imu_topic', c.imu_topic)
        self.declare_parameter('odom_topic', c.odom_topic)
        self.declare_parameter('alerts_topic', c.alerts_topic)
        self.declare_parameter('features_topic', c.vibe_features_topic)
        self.declare_parameter('status_topic', c.edge_status_topic)
        self.declare_parameter('vehicle_id', 'haul-01')
        self.declare_parameter('rms_warn', 1.5)
        self.declare_parameter('rms_crit', 2.5)
        self.declare_parameter('peak_warn', 4.0)
        self.declare_parameter('peak_crit', 6.0)
        self.declare_parameter('min_speed_mps', 1.0)
        self.declare_parameter('window_s', 1.0)
        self.declare_parameter('hop_s', 0.2)
        self.declare_parameter('hold_s', 0.8)
        self.declare_parameter('min_samples', 10)
        self.declare_parameter('max_gap_s', 0.25)
        self.declare_parameter('odom_timeout_s', 1.0)

        rms_warn = float(self.get_parameter('rms_warn').value)
        rms_crit = float(self.get_parameter('rms_crit').value)
        peak_warn = float(self.get_parameter('peak_warn').value)
        peak_crit = float(self.get_parameter('peak_crit').value)
        min_speed_mps = float(self.get_parameter('min_speed_mps').value)
        validate_thresholds(rms_warn, rms_crit, peak_warn, peak_crit, min_speed_mps)
        params = VibeParams(
            window_s=float(self.get_parameter('window_s').value),
            hop_s=float(self.get_parameter('hop_s').value),
            hold_s=float(self.get_parameter('hold_s').value),
            min_samples=int(self.get_parameter('min_samples').value),
            max_gap_s=float(self.get_parameter('max_gap_s').value),
            rms_warn=rms_warn,
            rms_crit=rms_crit,
            peak_warn=peak_warn,
            peak_crit=peak_crit,
            min_speed_mps=min_speed_mps,
        )
        self.monitor = VibrationMonitor(params)
        self.vehicle_id = str(self.get_parameter('vehicle_id').value)
        self._speed = 0.0
        self._pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._odom_received_at: Optional[float] = None
        self._odom_timeout_s = float(self.get_parameter('odom_timeout_s').value)
        if self._odom_timeout_s <= 0:
            raise ValueError('odom_timeout_s must be positive')
        self._samples = 0
        self._alerts = 0
        self._last_imu_t: Optional[float] = None
        self._last_feature_t: Optional[float] = None
        self._zero_stamp_warned = False

        imu_topic = str(self.get_parameter('imu_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        self.pub_alert = self.create_publisher(String, str(self.get_parameter('alerts_topic').value), 10)
        self.pub_feat = self.create_publisher(String, str(self.get_parameter('features_topic').value), 10)
        self.pub_status = self.create_publisher(String, str(self.get_parameter('status_topic').value), 10)
        self.create_subscription(Imu, imu_topic, self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_timer(2.0, self._heartbeat)
        self.get_logger().info(f'edge_vibe_detect imu={imu_topic} odom={odom_topic}')

    def _on_odom(self, msg: Odometry) -> None:
        tw = msg.twist.twist
        self._speed = float((tw.linear.x ** 2 + tw.linear.y ** 2) ** 0.5)
        p = msg.pose.pose.position
        self._pose = (float(p.x), float(p.y), float(p.z))
        self._odom_received_at = time.monotonic()

    def _on_imu(self, msg: Imu) -> None:
        sensor_t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        ros_now = float(self.get_clock().now().nanoseconds) * 1e-9
        if sensor_t <= 0.0:
            sensor_t = ros_now
            if not self._zero_stamp_warned:
                self.get_logger().warn('IMU stamp is zero; using ROS receive time')
                self._zero_stamp_warned = True
        self._last_imu_t = sensor_t
        sample = ImuSample(
            t=sensor_t,
            ax=float(msg.linear_acceleration.x),
            ay=float(msg.linear_acceleration.y),
            az=float(msg.linear_acceleration.z),
        )
        self._samples += 1
        feat = self.monitor.push(sample)
        if feat is None:
            return
        self._last_feature_t = feat.t
        fmsg = String()
        fmsg.data = json.dumps(self.monitor.features_dict(feat))
        self.pub_feat.publish(fmsg)

        odom_ok = is_fresh(self._odom_received_at, time.monotonic(), self._odom_timeout_s)
        alert = self.monitor.evaluate(
            feat,
            speed_mps=self._speed if odom_ok else float('nan'),
            vehicle_id=self.vehicle_id,
            source_node='edge_vibe_detect',
            pose=self._pose,
        )
        if alert is not None:
            alert.t_ros = ros_now
            amsg = String()
            amsg.data = json.dumps(alert.to_dict())
            self.pub_alert.publish(amsg)
            self._alerts += 1
            self.get_logger().warn(
                f'vibration alert severity={alert.severity} rms_az={alert.details.get("rms_az"):.2f}'
            )

    def _heartbeat(self) -> None:
        now = time.monotonic()
        odom_ok = is_fresh(self._odom_received_at, now, self._odom_timeout_s)
        odom_age = None if self._odom_received_at is None else max(0.0, now - self._odom_received_at)
        msg = String()
        msg.data = json.dumps(
            {
                'node': 'edge_vibe_detect',
                'vehicle_id': self.vehicle_id,
                'samples': self._samples,
                'alerts': self._alerts,
                'speed_mps': self._speed if odom_ok else None,
                'odom_ok': odom_ok,
                'odom_age_s': odom_age,
                'last_imu_t': self._last_imu_t,
                'last_feature_t': self._last_feature_t,
            }
        )
        self.pub_status.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = VibeDetectNode()
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
