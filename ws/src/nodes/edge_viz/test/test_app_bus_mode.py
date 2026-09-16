import asyncio

from aiohttp.test_utils import TestClient, TestServer

from edge_viz.app import STATIC_DIR, create_app
from edge_viz.tick_buffer import TickBuffer


def test_bus_mode_config_endpoint_and_local_assets():
    async def scenario():
        buffer = TickBuffer()
        app = create_app(buffer, ws_hz=10.0)
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            response = await client.get('/api/config')
            assert response.status == 200
            payload = await response.json()
            assert payload['mode'] == 'bus'
            assert payload['duration_s'] == buffer.cfg.history_s
            assert payload['lidar_hz'] == 0.0
        finally:
            await client.close()

    asyncio.run(scenario())
    html = (STATIC_DIR / 'index.html').read_text()
    assert 'https://cdn.jsdelivr.net' not in html
    assert '/static/three.module.js' in html
    assert '/static/chart.umd.js' in html
