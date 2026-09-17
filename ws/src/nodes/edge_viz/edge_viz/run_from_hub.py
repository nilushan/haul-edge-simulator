#!/usr/bin/env python3
"""
Sole process entry: one StreamHub + optional viz HTTP + optional ROS bridge.

Visualizer and ROS both *subscribe* to the same hub. Nothing else synthesizes sensors.
"""

from __future__ import annotations

import argparse
import logging
import os
import threading
from typing import List, Optional

from edge_viz.app import build_hub_from_args, create_app
from edge_viz.perception_overlay import PerceptionOverlay
from edge_sim.maps import DEFAULT_PLAYLIST
from edge_sim.stream import StreamHub

log = logging.getLogger('edge_viz.hub')


def _start_ros_bridge(hub: StreamHub) -> threading.Thread:
    """Spin sensor source node against the shared hub in a background thread."""

    def _run() -> None:
        try:
            import rclpy
            from edge_sensor_source.source_node import SensorSourceNode
        except ImportError as exc:
            log.error('ROS bridge requested but rclpy unavailable: %s', exc)
            return

        rclpy.init()
        node = SensorSourceNode(hub=hub)
        try:
            rclpy.spin(node)
        except Exception:  # noqa: BLE001
            log.exception('ROS bridge stopped')
        finally:
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
            if rclpy.ok():
                rclpy.shutdown()

    t = threading.Thread(target=_run, name='ros-bridge', daemon=True)
    t.start()
    return t


def main(argv: Optional[List[str]] = None) -> None:
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    p = argparse.ArgumentParser(description='Sole sensor stream server (hub + viz [+ ROS])')
    p.add_argument('--host', default=os.environ.get('VIZ_HOST', '0.0.0.0'))
    p.add_argument('--port', type=int, default=int(os.environ.get('VIZ_PORT', '8099')))
    p.add_argument('--duration', type=float, default=float(os.environ.get('DURATION', '600')))
    p.add_argument('--no-loop', action='store_true')
    p.add_argument('--cycle', action='store_true', help='advance map/stream each duration window')
    p.add_argument('--mode', choices=('live', 'replay'), default=os.environ.get('STREAM_MODE', 'live'))
    p.add_argument('--maps', default=os.environ.get('MAPS', ','.join(DEFAULT_PLAYLIST)))
    p.add_argument('--stream', default=os.environ.get('STREAM_PATH', ''))
    p.add_argument('--streams', default=os.environ.get('STREAM_IDS', ''))
    p.add_argument('--streams-root', default=os.environ.get('STREAMS_ROOT', ''))
    p.add_argument('--record-dir', default=os.environ.get('RECORD_DIR', ''))
    p.add_argument('--imu-hz', type=float, default=50.0)
    p.add_argument('--gnss-hz', type=float, default=5.0)
    p.add_argument('--lidar-hz', type=float, default=10.0)
    p.add_argument('--ws-hz', type=float, default=10.0)
    p.add_argument('--speed', type=float, default=None)
    p.add_argument('--ros', action='store_true', help='also publish ROS topics from the same hub')
    p.add_argument('--no-viz', action='store_true', help='hub + optional ROS only (no HTTP UI)')
    p.add_argument(
        '--no-inline-detect',
        action='store_true',
        help='serve raw hub ticks without the inline perception pass',
    )
    args = p.parse_args(argv)

    args.stream = args.stream or None
    args.record_dir = args.record_dir or None
    args.streams_root = args.streams_root or None

    # Env MODE=all implies --ros
    if os.environ.get('MODE', '').lower() in {'all', 'viz+ros', 'ros+viz'}:
        args.ros = True
    if os.environ.get('MODE', '').lower() == 'ros':
        args.ros = True
        args.no_viz = True

    hub = build_hub_from_args(args)
    cat = hub.catalog()
    log.info(
        'Sole StreamHub mode=%s playlist_maps=%s recorded_streams=%d source=%s',
        hub.cfg.mode,
        cat['playlist']['maps'],
        len(cat['streams']),
        hub.snapshot().get('source'),
    )

    ros_thread = None
    if args.ros:
        log.info('Starting ROS bridge subscriber on shared hub')
        # Hub must be running before ROS polls it
        hub.start()
        ros_thread = _start_ros_bridge(hub)

    if args.no_viz:
        if not args.ros:
            hub.start()
        log.info('Running hub without viz (Ctrl+C to stop)')
        try:
            import time
            while True:
                snap = hub.snapshot()
                if not snap.get('running') and not args.ros:
                    break
                time.sleep(0.5)
                if ros_thread and not ros_thread.is_alive():
                    break
        except KeyboardInterrupt:
            pass
        finally:
            hub.stop()
        return

    # Viz owns hub start via app startup if not already started
    if args.ros:
        # already started; create_app will call hub.start() again (idempotent)
        pass

    from aiohttp import web

    # Viz subscribes through the perception overlay; ROS still reads the raw hub.
    source = hub if args.no_inline_detect else PerceptionOverlay(hub)
    app = create_app(source, ws_hz=args.ws_hz)
    log.info('Visualizer subscribing at http://%s:%s/', args.host, args.port)
    try:
        web.run_app(app, host=args.host, port=args.port, print=None)
    finally:
        hub.stop()


if __name__ == '__main__':
    main()
