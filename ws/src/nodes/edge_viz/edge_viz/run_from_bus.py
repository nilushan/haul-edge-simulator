#!/usr/bin/env python3
"""
Edge visualizer that subscribes to the ROS sensor bus (not StreamHub).

Use this when you want the UI to behave like production:
  any source on the bus (sim, bag, real drivers) → same viz.

  python3 -m edge_viz.run_from_bus --port 8099
"""

from __future__ import annotations

import argparse
import logging
import os
import threading
from typing import List, Optional

from edge_viz.bus_ingress import RosBusIngress
from edge_viz.app import create_app
from edge_sim.topics import SensorBusContract
from edge_viz.tick_buffer import TickBuffer, TickBufferConfig

log = logging.getLogger('edge_viz.bus')


class _BusSource:
    """Thin adapter: TickBuffer + ROS spin thread, hub-compatible API for create_app."""

    def __init__(self, buffer: TickBuffer) -> None:
        self.buffer = buffer
        self.cfg = buffer.cfg
        self._node = None
        self._thread: Optional[threading.Thread] = None
        self._rclpy = None

    def start(self) -> None:
        self.buffer.start()
        try:
            import rclpy
            from rclpy.node import Node
        except ImportError as exc:
            raise RuntimeError('viz_bus_server requires rclpy (ROS 2)') from exc

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()

        node = Node('edge_viz_bus')
        self._node = node
        RosBusIngress(node, self.buffer, contract=self.buffer.contract)

        def _spin() -> None:
            try:
                rclpy.spin(node)
            except Exception:  # noqa: BLE001
                log.exception('ROS spin ended')

        self._thread = threading.Thread(target=_spin, name='viz-bus-spin', daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.buffer.stop()
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        if self._rclpy is not None and self._rclpy.ok():
            try:
                self._rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass

    def snapshot(self):
        return self.buffer.snapshot()

    def live_tick(self):
        return self.buffer.live_tick()

    def catalog(self):
        return self.buffer.catalog()

    def set_map(self, map_id: str) -> None:
        self.buffer.set_map(map_id)

    def set_stream(self, stream_id: str) -> None:
        self.buffer.set_stream(stream_id)


def _argv_without_ros(argv: Optional[List[str]] = None) -> List[str]:
    """Drop launch-injected --ros-args so argparse can run under ros2 launch."""
    import sys

    raw = list(sys.argv[1:] if argv is None else argv)
    try:
        from rclpy.utilities import remove_ros_args

        cleaned = remove_ros_args(args=['viz_from_bus', *raw])
        return list(cleaned[1:]) if cleaned else []
    except Exception:  # noqa: BLE001
        if '--ros-args' in raw:
            return raw[: raw.index('--ros-args')]
        return raw


def main(argv: Optional[List[str]] = None) -> None:
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    p = argparse.ArgumentParser(description='Visualizer subscribed to ROS sensor bus')
    p.add_argument('--host', default=os.environ.get('VIZ_HOST', '0.0.0.0'))
    p.add_argument('--port', type=int, default=int(os.environ.get('VIZ_PORT', '8099')))
    p.add_argument('--ws-hz', type=float, default=10.0)
    p.add_argument('--history', type=float, default=60.0)
    p.add_argument('--imu-topic', default='')
    p.add_argument('--gnss-topic', default='')
    p.add_argument('--lidar-topic', default='')
    p.add_argument('--odom-topic', default='')
    args = p.parse_args(_argv_without_ros(argv))

    contract = SensorBusContract()
    # optional overrides
    updates = {}
    if args.imu_topic:
        updates['imu_topic'] = args.imu_topic
    if args.gnss_topic:
        updates['gnss_topic'] = args.gnss_topic
    if args.lidar_topic:
        updates['lidar_topic'] = args.lidar_topic
    if args.odom_topic:
        updates['odom_topic'] = args.odom_topic
    if updates:
        contract = SensorBusContract.from_dict({**contract.to_dict(), **updates})

    buf = TickBuffer(
        TickBufferConfig(history_s=args.history, source_label='bus'),
        contract=contract,
    )
    source = _BusSource(buf)

    from aiohttp import web

    app = create_app(source, ws_hz=args.ws_hz)  # type: ignore[arg-type]
    log.info(
        'Viz bus mode — subscribing to ROS topics %s | http://%s:%s/',
        contract.imu_topic,
        args.host,
        args.port,
    )
    try:
        web.run_app(app, host=args.host, port=args.port, print=None)
    finally:
        source.stop()


if __name__ == '__main__':
    main()
