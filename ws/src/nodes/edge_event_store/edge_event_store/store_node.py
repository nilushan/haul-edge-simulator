#!/usr/bin/env python3
"""Persist /edge/alerts to local SQLite (edge storage before cloud sync)."""

from __future__ import annotations

import json
import os
import sqlite3
import time
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from edge_sim.topics import SensorBusContract


class EventStoreNode(Node):
    def __init__(self) -> None:
        super().__init__('edge_event_store')
        c = SensorBusContract()
        default_db = os.environ.get(
            'EDGE_EVENT_DB',
            str(Path.home() / '.cache' / 'haul-edge' / 'events.sqlite'),
        )
        self.declare_parameter('alerts_topic', c.alerts_topic)
        self.declare_parameter('status_topic', c.edge_status_topic)
        self.declare_parameter('db_path', default_db)

        self.db_path = Path(str(self.get_parameter('db_path').value))
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(str(self.db_path), check_same_thread=False)
        self._conn.execute(
            """
            CREATE TABLE IF NOT EXISTS events (
              id TEXT PRIMARY KEY,
              t REAL NOT NULL,
              type TEXT NOT NULL,
              severity TEXT,
              json TEXT NOT NULL,
              synced INTEGER NOT NULL DEFAULT 0,
              created_at REAL NOT NULL
            )
            """
        )
        self._conn.execute('CREATE INDEX IF NOT EXISTS idx_events_t ON events(t)')
        self._conn.execute('CREATE INDEX IF NOT EXISTS idx_events_synced ON events(synced)')
        self._conn.commit()

        self._stored = 0
        self.pub_status = self.create_publisher(String, str(self.get_parameter('status_topic').value), 10)
        topic = str(self.get_parameter('alerts_topic').value)
        self.create_subscription(String, topic, self._on_alert, 50)
        self.create_timer(5.0, self._heartbeat)
        self.get_logger().info(f'edge_event_store db={self.db_path} sub={topic}')

    def _on_alert(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('ignoring non-JSON alert')
            return
        event_id = str(payload.get('event_id') or f'evt-{time.time()}')
        t = float(payload.get('t_ros') or payload.get('t_vehicle') or time.time())
        etype = str(payload.get('type') or 'unknown')
        severity = str(payload.get('severity') or '')
        try:
            self._conn.execute(
                'INSERT OR IGNORE INTO events (id, t, type, severity, json, synced, created_at) VALUES (?,?,?,?,?,?,?)',
                (event_id, t, etype, severity, json.dumps(payload), 0, time.time()),
            )
            self._conn.commit()
            self._stored += 1
        except sqlite3.Error as exc:
            self.get_logger().error(f'db write failed: {exc}')

    def _heartbeat(self) -> None:
        cur = self._conn.execute('SELECT COUNT(*), SUM(CASE WHEN synced=0 THEN 1 ELSE 0 END) FROM events')
        total, unsynced = cur.fetchone()
        msg = String()
        msg.data = json.dumps(
            {
                'node': 'edge_event_store',
                'db': str(self.db_path),
                'stored_session': self._stored,
                'total': int(total or 0),
                'unsynced': int(unsynced or 0),
            }
        )
        self.pub_status.publish(msg)

    def destroy_node(self) -> bool:
        try:
            self._conn.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = EventStoreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
