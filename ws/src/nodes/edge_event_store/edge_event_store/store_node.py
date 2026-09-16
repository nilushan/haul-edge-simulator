#!/usr/bin/env python3
"""Persist /edge/alerts to bounded local SQLite edge storage."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from edge_event_store.storage import EventStore
from edge_sim.topics import SensorBusContract


class EventStoreNode(Node):
    def __init__(self) -> None:
        super().__init__('edge_event_store')
        contract = SensorBusContract()
        default_db = os.environ.get(
            'EDGE_EVENT_DB',
            str(Path.home() / '.cache' / 'haul-edge' / 'events.sqlite'),
        )
        self.declare_parameter('alerts_topic', contract.alerts_topic)
        self.declare_parameter('status_topic', contract.edge_status_topic)
        self.declare_parameter('db_path', default_db)
        self.declare_parameter('max_rows', 100000)
        self.declare_parameter('max_age_days', 30.0)

        self.db_path = Path(str(self.get_parameter('db_path').value))
        self.max_rows = int(self.get_parameter('max_rows').value)
        self.max_age_s = float(self.get_parameter('max_age_days').value) * 86400.0
        if self.max_rows < 1 or self.max_age_s <= 0:
            raise ValueError('max_rows and max_age_days must be positive')
        self._store = EventStore(self.db_path)
        self._stored = 0
        self._rejected = 0
        self._pruned = 0

        self.pub_status = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )
        topic = str(self.get_parameter('alerts_topic').value)
        self.create_subscription(String, topic, self._on_alert, 50)
        self.create_timer(5.0, self._heartbeat)
        self.get_logger().info(f'edge_event_store db={self.db_path} sub={topic}')

    def _on_alert(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            inserted = self._store.insert(payload)
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self._rejected += 1
            self.get_logger().warn(f'ignoring malformed alert: {exc}')
            return
        except Exception as exc:  # SQLite/storage failures must not kill the executor
            self._rejected += 1
            self.get_logger().error(f'db write failed: {exc}')
            return
        if inserted:
            self._stored += 1

    def _heartbeat(self) -> None:
        try:
            self._pruned += self._store.prune(
                max_rows=self.max_rows,
                max_age_s=self.max_age_s,
            )
            total = self._store.count()
        except Exception as exc:
            self.get_logger().error(f'db maintenance failed: {exc}')
            total = -1
        msg = String()
        msg.data = json.dumps(
            {
                'node': 'edge_event_store',
                'db': str(self.db_path),
                'stored_session': self._stored,
                'rejected_session': self._rejected,
                'pruned_session': self._pruned,
                'total': total,
                'max_rows': self.max_rows,
                'max_age_s': self.max_age_s,
            }
        )
        self.pub_status.publish(msg)

    def destroy_node(self) -> bool:
        try:
            self._store.close()
        except Exception as exc:
            self.get_logger().error(f'db close failed: {exc}')
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = EventStoreNode()
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
