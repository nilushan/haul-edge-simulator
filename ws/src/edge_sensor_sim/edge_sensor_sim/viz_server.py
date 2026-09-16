#!/usr/bin/env python3
"""HTTP REST + WebSocket visualizer for the haul-edge sensor suite."""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
from pathlib import Path
from typing import Set

from aiohttp import WSMsgType, web

from edge_sensor_sim.suite_runner import SuiteConfig, SuiteRunner

log = logging.getLogger('edge_sensor_sim.viz')

STATIC_DIR = Path(__file__).resolve().parent / 'static'


def create_app(runner: SuiteRunner, ws_hz: float = 10.0) -> web.Application:
    app = web.Application()
    app['runner'] = runner
    app['ws_clients']: Set[web.WebSocketResponse] = set()
    app['ws_hz'] = ws_hz

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
        snap = runner.snapshot()
        return web.json_response({'ok': True, 't': snap['t'], 'running': snap['running']})

    async def api_status(_: web.Request) -> web.Response:
        snap = runner.snapshot()
        # drop bulky lidar xy from status
        slim = {k: v for k, v in snap.items() if k != 'lidar'}
        slim['lidar'] = {'n': (snap.get('lidar') or {}).get('n'), 't': (snap.get('lidar') or {}).get('t')}
        slim['history_counts'] = {
            'imu': len(snap['history']['imu']),
            'gnss': len(snap['history']['gnss']),
            'odom': len(snap['history']['odom']),
        }
        # still include short tails for initial chart paint
        slim['history'] = {
            'imu': snap['history']['imu'][-300:],
            'gnss': snap['history']['gnss'][-120:],
            'odom': snap['history']['odom'][-600:],
        }
        return web.json_response(slim)

    async def api_history(request: web.Request) -> web.Response:
        kind = request.match_info.get('kind', 'imu')
        snap = runner.snapshot()
        if kind not in snap['history']:
            raise web.HTTPNotFound(text=f'unknown history kind {kind}')
        return web.json_response({'kind': kind, 'samples': snap['history'][kind]})

    async def api_lidar(_: web.Request) -> web.Response:
        snap = runner.snapshot()
        return web.json_response(snap.get('lidar') or {})

    async def api_config(_: web.Request) -> web.Response:
        c = runner.cfg
        return web.json_response({
            'duration_s': c.duration_s,
            'loop': c.loop,
            'imu_hz': c.imu_hz,
            'gnss_hz': c.gnss_hz,
            'lidar_hz': c.lidar_hz,
            'speed_mps': c.speed_mps,
            'history_s': c.history_s,
        })

    async def ws_stream(request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=20.0)
        await ws.prepare(request)
        app['ws_clients'].add(ws)
        try:
            # initial full-ish tick
            await ws.send_json(runner.live_tick())
            async for msg in ws:
                if msg.type in (WSMsgType.CLOSE, WSMsgType.ERROR):
                    break
                # client may send {"cmd":"ping"}
                if msg.type == WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                    except json.JSONDecodeError:
                        continue
                    if data.get('cmd') == 'snapshot':
                        await ws.send_json({'type': 'snapshot', **runner.snapshot()})
        finally:
            app['ws_clients'].discard(ws)
        return ws

    app.router.add_get('/', index)
    app.router.add_get('/static/{name}', static_file)
    app.router.add_get('/api/healthz', healthz)
    app.router.add_get('/api/status', api_status)
    app.router.add_get('/api/config', api_config)
    app.router.add_get('/api/history/{kind}', api_history)
    app.router.add_get('/api/lidar', api_lidar)
    app.router.add_get('/ws', ws_stream)

    async def on_start(app: web.Application) -> None:
        runner.start()

        async def pump() -> None:
            period = 1.0 / max(1.0, float(app['ws_hz']))
            while True:
                await asyncio.sleep(period)
                payload = runner.live_tick()
                dead = []
                for ws in list(app['ws_clients']):
                    try:
                        await ws.send_json(payload)
                    except ConnectionResetError:
                        dead.append(ws)
                    except Exception:  # noqa: BLE001
                        dead.append(ws)
                for ws in dead:
                    app['ws_clients'].discard(ws)

        app['pump_task'] = asyncio.create_task(pump())

    async def on_stop(app: web.Application) -> None:
        task = app.get('pump_task')
        if task:
            task.cancel()
        runner.stop()

    app.on_startup.append(on_start)
    app.on_cleanup.append(on_stop)
    return app


def main(argv: list[str] | None = None) -> None:
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    p = argparse.ArgumentParser(description='Haul-edge sensor suite visualizer')
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=8099)
    p.add_argument('--duration', type=float, default=60.0, help='sim cycle length seconds')
    p.add_argument('--no-loop', action='store_true')
    p.add_argument('--imu-hz', type=float, default=50.0)
    p.add_argument('--gnss-hz', type=float, default=5.0)
    p.add_argument('--lidar-hz', type=float, default=5.0)
    p.add_argument('--ws-hz', type=float, default=10.0)
    p.add_argument('--speed', type=float, default=8.0)
    args = p.parse_args(argv)

    cfg = SuiteConfig(
        duration_s=args.duration,
        loop=not args.no_loop,
        imu_hz=args.imu_hz,
        gnss_hz=args.gnss_hz,
        lidar_hz=args.lidar_hz,
        speed_mps=args.speed,
        history_s=args.duration,
    )
    runner = SuiteRunner(cfg)
    app = create_app(runner, ws_hz=args.ws_hz)
    log.info('Open http://%s:%s/  (duration=%.0fs loop=%s)', args.host, args.port, cfg.duration_s, cfg.loop)
    web.run_app(app, host=args.host, port=args.port, print=None)


if __name__ == '__main__':
    main()
