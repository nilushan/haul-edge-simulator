#!/usr/bin/env python3
"""IMU vibration detector: /imu/data + /odom → /edge/alerts."""

from __future__ import annotations

import json
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from edge_perception.vibration import ImuSample, VibrationMonitor, VibeParams
from edge_sim.topics import SensorBusContract


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

        params = VibeParams(
            rms_warn=float(self.get_parameter('rms_warn').value),
            rms_crit=float(self.get_parameter('rms_crit').value),
            peak_warn=float(self.get_parameter('peak_warn').value),
            peak_crit=float(self.get_parameter('peak_crit').value),
            min_speed_mps=float(self.get_parameter('min_speed_mps').value),
        )
        self.monitor = VibrationMonitor(params)
        self.vehicle_id = str(self.get_parameter('vehicle_id').value)
        self._speed = 0.0
        self._pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._samples = 0
        self._alerts = 0

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

    def _on_imu(self, msg: Imu) -> None:
        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        sample = ImuSample(
            t=t,
            ax=float(msg.linear_acceleration.x),
            ay=float(msg.linear_acceleration.y),
            az=float(msg.linear_acceleration.z),
        )
        self._samples += 1
        feat = self.monitor.push(sample)
        if feat is None:
            return
        fmsg = String()
        fmsg.data = json.dumps(self.monitor.features_dict(feat))
        self.pub_feat.publish(fmsg)

        alert = self.monitor.evaluate(
            feat,
            speed_mps=self._speed,
            vehicle_id=self.vehicle_id,
            source_node='edge_vibe_detect',
            pose=self._pose,
        )
        if alert is not None:
            alert.t_ros = t
            amsg = String()
            amsg.data = json.dumps(alert.to_dict())
            self.pub_alert.publish(amsg)
            self._alerts += 1
            self.get_logger().warn(
                f'vibration alert severity={alert.severity} rms_az={alert.details.get("rms_az"):.2f}'
            )

    def _heartbeat(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                'node': 'edge_vibe_detect',
                'samples': self._samples,
                'alerts': self._alerts,
                'speed_mps': self._speed,
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
