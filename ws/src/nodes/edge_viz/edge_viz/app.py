#!/usr/bin/env python3
"""HTTP REST + WebSocket visualizer — subscribes to StreamHub only (no models)."""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import math
import os
from pathlib import Path
from typing import Any, List, Optional, Set

from aiohttp import WSMsgType, web

from edge_perception.schema import style_catalog
from edge_sim.maps import DEFAULT_PLAYLIST
from edge_sim.stream import default_streams_root
from edge_sim.stream import StreamConfig, StreamHub
from edge_viz.perception_overlay import PerceptionOverlay

log = logging.getLogger('edge_viz')

STATIC_DIR = Path(__file__).resolve().parent / 'static'


def create_app(hub: Any, ws_hz: float = 10.0) -> web.Application:
    """Build aiohttp app that only *reads* a tick source (StreamHub or bus buffer)."""
    if not math.isfinite(ws_hz) or ws_hz <= 0:
        raise ValueError('ws_hz must be positive and finite')
    app = web.Application()
    ws_clients: Set[web.WebSocketResponse] = set()
    pump_task: Optional[asyncio.Task[None]] = None

    async def index(_: web.Request) -> web.FileResponse:
        return web.FileResponse(STATIC_DIR / 'index.html')

    async def static_file(request: web.Request) -> web.FileResponse:
        name = request.match_info['name']
        path = STATIC_DIR / name
        if not path.is_file() or path.suffix not in {'.js', '.css', '.html', '.svg'}:
            raise web.HTTPNotFound()
        resp = web.FileResponse(path)
        resp.headers['Cache-Control'] = 'no-store'
        return resp

    async def healthz(_: web.Request) -> web.Response:
        snap = hub.snapshot()
        return web.json_response(
            {
                'ok': True,
                't': snap['t'],
                'running': snap['running'],
                'source': snap.get('source'),
                'map_id': snap.get('map_id'),
            }
        )

    async def api_status(_: web.Request) -> web.Response:
        snap = hub.snapshot()
        slim = {k: v for k, v in snap.items() if k != 'lidar'}
        slim['lidar'] = {
            'n': (snap.get('lidar') or {}).get('n'),
            't': (snap.get('lidar') or {}).get('t'),
        }
        slim['history_counts'] = {
            'imu': len(snap['history']['imu']),
            'gnss': len(snap['history']['gnss']),
            'odom': len(snap['history']['odom']),
        }
        slim['history'] = {
            'imu': snap['history']['imu'][-300:],
            'gnss': snap['history']['gnss'][-120:],
            'odom': snap['history']['odom'][-600:],
        }
        return web.json_response(slim)

    async def api_history(request: web.Request) -> web.Response:
        kind = request.match_info.get('kind', 'imu')
        snap = hub.snapshot()
        if kind not in snap['history']:
            raise web.HTTPNotFound(text=f'unknown history kind {kind}')
        return web.json_response({'kind': kind, 'samples': snap['history'][kind]})

    async def api_lidar(_: web.Request) -> web.Response:
        snap = hub.snapshot()
        return web.json_response(snap.get('lidar') or {})

    async def api_config(_: web.Request) -> web.Response:
        c = hub.cfg
        snap = hub.snapshot()
        return web.json_response(
            {
                'duration_s': getattr(c, 'duration_s', snap.get('duration_s', 0.0)),
                'loop': getattr(c, 'loop', snap.get('loop', True)),
                'imu_hz': getattr(c, 'imu_hz', 0.0),
                'gnss_hz': getattr(c, 'gnss_hz', 0.0),
                'lidar_hz': getattr(c, 'lidar_hz', 0.0),
                'mode': getattr(c, 'mode', snap.get('mode', 'bus')),
                'cycle_maps': getattr(c, 'cycle_maps', False),
                'map_id': snap.get('map_id'),
                'stream_id': snap.get('stream_id'),
                'source': snap.get('source'),
                'history_s': getattr(c, 'history_s', snap.get('duration_s', 0.0)),
            }
        )

    async def api_classes(_: web.Request) -> web.Response:
        """Class/event style table — single source of colours and labels."""
        return web.json_response(style_catalog())

    async def api_catalog(_: web.Request) -> web.Response:
        return web.json_response(hub.catalog())

    async def api_select(request: web.Request) -> web.Response:
        """POST {"map_id": "..."} or {"stream_id": "..."} to switch source."""
        try:
            body = await request.json()
        except json.JSONDecodeError as exc:
            raise web.HTTPBadRequest(text='invalid JSON') from exc
        if not isinstance(body, dict):
            raise web.HTTPBadRequest(text='JSON body must be an object')
        try:
            if 'map_id' in body and body['map_id']:
                hub.set_map(str(body['map_id']))
            elif 'stream_id' in body and body['stream_id']:
                hub.set_stream(str(body['stream_id']))
            else:
                raise web.HTTPBadRequest(text='need map_id or stream_id')
        except (KeyError, FileNotFoundError, RuntimeError) as exc:
            raise web.HTTPBadRequest(text=str(exc)) from exc
        return web.json_response(hub.catalog())

    async def ws_stream(request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=20.0)
        await ws.prepare(request)
        ws_clients.add(ws)
        try:
            # Initial tick from hub (subscriber read)
            await ws.send_json(hub.live_tick())
            async for msg in ws:
                if msg.type in (WSMsgType.CLOSE, WSMsgType.ERROR):
                    break
                if msg.type == WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                    except json.JSONDecodeError:
                        continue
                    if not isinstance(data, dict):
                        continue
                    if data.get('cmd') == 'snapshot':
                        await ws.send_json({'type': 'snapshot', **hub.snapshot()})
                    elif data.get('cmd') == 'select_map' and data.get('map_id'):
                        try:
                            hub.set_map(str(data['map_id']))
                            await ws.send_json({'type': 'catalog', **hub.catalog()})
                        except (KeyError, RuntimeError) as exc:
                            await ws.send_json({'type': 'error', 'error': str(exc)})
                    elif data.get('cmd') == 'select_stream' and data.get('stream_id'):
                        try:
                            hub.set_stream(str(data['stream_id']))
                            await ws.send_json({'type': 'catalog', **hub.catalog()})
                        except (KeyError, FileNotFoundError, RuntimeError) as exc:
                            await ws.send_json({'type': 'error', 'error': str(exc)})
        finally:
            ws_clients.discard(ws)
        return ws

    app.router.add_get('/', index)
    app.router.add_get('/static/{name}', static_file)
    app.router.add_get('/api/healthz', healthz)
    app.router.add_get('/api/status', api_status)
    app.router.add_get('/api/config', api_config)
    app.router.add_get('/api/catalog', api_catalog)
    app.router.add_get('/api/classes', api_classes)
    app.router.add_post('/api/select', api_select)
    app.router.add_get('/api/history/{kind}', api_history)
    app.router.add_get('/api/lidar', api_lidar)
    app.router.add_get('/ws', ws_stream)

    async def on_start(_: web.Application) -> None:
        nonlocal pump_task
        # Hub is the sole generator; viz only pumps subscribed ticks to browsers.
        hub.start()

        async def pump() -> None:
            period = 1.0 / max(1.0, float(ws_hz))
            while True:
                await asyncio.sleep(period)
                payload = hub.live_tick()  # subscribe/read — never synthesize here
                dead = []
                for ws in list(ws_clients):
                    try:
                        await ws.send_json(payload)
                    except ConnectionResetError:
                        dead.append(ws)
                    except Exception:  # noqa: BLE001
                        dead.append(ws)
                for ws in dead:
                    ws_clients.discard(ws)

        pump_task = asyncio.create_task(pump())

    async def on_stop(_: web.Application) -> None:
        if pump_task:
            pump_task.cancel()
            try:
                await pump_task
            except asyncio.CancelledError:
                pass
        for ws in list(ws_clients):
            await ws.close(code=1001, message=b'server shutdown')
        hub.stop()

    app.on_startup.append(on_start)
    app.on_cleanup.append(on_stop)
    return app


def _parse_list(value: Optional[str]) -> List[str]:
    if not value:
        return []
    return [p.strip() for p in value.split(',') if p.strip()]


def build_hub_from_args(args: argparse.Namespace) -> StreamHub:
    mode = args.mode
    map_ids = _parse_list(args.maps) or list(DEFAULT_PLAYLIST)
    stream_ids = _parse_list(getattr(args, 'streams', '') or '')
    streams_root = args.streams_root or str(default_streams_root())

    if args.stream:
        mode = 'replay'

    cfg = StreamConfig(
        mode=mode,
        map_ids=map_ids,
        stream_path=args.stream,
        streams_root=streams_root,
        stream_ids=stream_ids,
        duration_s=args.duration,
        loop=not args.no_loop,
        cycle_maps=bool(getattr(args, 'cycle', False)),
        imu_hz=args.imu_hz,
        gnss_hz=args.gnss_hz,
        lidar_hz=args.lidar_hz,
        history_s=args.duration,
        speed_mps=args.speed,
        record_dir=args.record_dir,
    )
    return StreamHub(cfg)


def build_source_from_args(args: argparse.Namespace) -> Any:
    """Hub, wrapped with inline perception unless the caller opted out."""
    hub = build_hub_from_args(args)
    if getattr(args, 'no_inline_detect', False):
        return hub
    return PerceptionOverlay(hub)


def main(argv: list[str] | None = None) -> None:
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    p = argparse.ArgumentParser(
        description='Live visualizer for simulated haul-truck sensors and perception'
    )
    p.add_argument('--host', default=os.environ.get('VIZ_HOST', '127.0.0.1'))
    p.add_argument('--port', type=int, default=int(os.environ.get('VIZ_PORT', '8099')))
    p.add_argument('--duration', type=float, default=float(os.environ.get('DURATION', '600')))
    p.add_argument('--no-loop', action='store_true')
    p.add_argument('--cycle', action='store_true', help='advance map/stream each duration window')
    p.add_argument('--mode', choices=('live', 'replay'), default=os.environ.get('STREAM_MODE', 'live'))
    p.add_argument(
        '--maps',
        default=os.environ.get('MAPS', ','.join(DEFAULT_PLAYLIST)),
        help='comma-separated map playlist for live mode',
    )
    p.add_argument('--stream', default=os.environ.get('STREAM_PATH', ''), help='replay one stream directory')
    p.add_argument(
        '--streams',
        default=os.environ.get('STREAM_IDS', ''),
        help='comma-separated recorded stream ids to playlist',
    )
    p.add_argument(
        '--streams-root',
        default=os.environ.get('STREAMS_ROOT', ''),
        help='catalog root (default: data/streams)',
    )
    p.add_argument('--record-dir', default=os.environ.get('RECORD_DIR', ''), help='record live maps under this dir')
    p.add_argument('--imu-hz', type=float, default=50.0)
    p.add_argument('--gnss-hz', type=float, default=5.0)
    p.add_argument('--lidar-hz', type=float, default=10.0)
    p.add_argument('--ws-hz', type=float, default=10.0)
    p.add_argument('--speed', type=float, default=None)
    p.add_argument(
        '--no-inline-detect',
        action='store_true',
        help='serve raw hub ticks without the inline perception pass',
    )
    args = p.parse_args(argv)

    # Normalize empty optionals
    args.stream = args.stream or None
    args.record_dir = args.record_dir or None
    args.streams_root = args.streams_root or None

    source = build_source_from_args(args)
    app = create_app(source, ws_hz=args.ws_hz)

    cat = source.catalog()
    log.info(
        'StreamHub mode=%s maps=%s streams=%d | Open http://%s:%s/',
        source.cfg.mode,
        [m['id'] for m in cat['maps']],
        len(cat['streams']),
        args.host,
        args.port,
    )
    web.run_app(app, host=args.host, port=args.port, print=None)


if __name__ == '__main__':
    main()
