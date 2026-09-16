"""Tests for multi-map StreamHub and replay format."""

from __future__ import annotations

import time
from pathlib import Path

from edge_sim.maps import MAPS, get_map, list_maps
from edge_sim.stream.format import StreamReader, discover_streams
from edge_sim.stream.hub import StreamConfig, StreamHub, record_map_stream


def test_map_catalog():
    maps = list_maps()
    assert len(maps) >= 4
    assert get_map('haul_corridor').id == 'haul_corridor'
    assert 'rocky_descent' in MAPS


def test_maps_differ():
    a = get_map('haul_corridor').build_world()
    b = get_map('tight_switchbacks').build_world()
    assert len(a.rocks) != len(b.rocks) or a.road_hw != b.road_hw
    x0, y0, _ = a.centerline(100.0)
    x1, y1, _ = b.centerline(100.0)
    assert abs(x0 - x1) + abs(y0 - y1) > 1.0


def test_live_hub_produces_samples():
    hub = StreamHub(
        StreamConfig(
            mode='live',
            map_ids=['haul_corridor'],
            duration_s=2.0,
            loop=True,
            cycle_maps=False,
            imu_hz=20.0,
            gnss_hz=5.0,
            lidar_hz=2.0,
            vehicle_hz=20.0,
            history_s=2.0,
        )
    )
    hub.start()
    time.sleep(0.8)
    snap = hub.snapshot()
    hub.stop()
    assert snap['running'] or snap['t'] > 0
    assert snap['latest']['odom'] is not None
    assert snap['latest']['imu'] is not None
    assert snap.get('lidar') is not None
    assert snap['map_id'] == 'haul_corridor'
    assert snap['mode'] == 'live'


def test_record_and_replay(tmp_path: Path):
    out = tmp_path / 'haul_corridor'
    record_map_stream(
        'haul_corridor',
        out,
        duration_s=1.0,
        imu_hz=20.0,
        gnss_hz=5.0,
        lidar_hz=2.0,
        vehicle_hz=20.0,
        lidar_max_points=500,
    )
    reader = StreamReader(out)
    assert reader.manifest.map_id == 'haul_corridor'
    assert len(reader.imu) >= 10
    assert len(reader.odom) >= 10
    assert len(reader.lidar) >= 1

    found = discover_streams(tmp_path)
    assert any(s['id'] == 'haul_corridor' for s in found)

    hub = StreamHub(
        StreamConfig(
            mode='replay',
            stream_path=str(out),
            duration_s=1.0,
            loop=True,
            cycle_maps=False,
            vehicle_hz=20.0,
            history_s=1.0,
        )
    )
    hub.start()
    time.sleep(0.6)
    tick = hub.live_tick()
    hub.stop()
    assert tick['mode'] == 'replay'
    assert tick['odom'] is not None or tick['imu'] is not None


def test_viz_subscribes_only_via_hub_api():
    hub = StreamHub(
        StreamConfig(
            mode='live',
            map_ids=['open_pit_bench'],
            duration_s=1.0,
            cycle_maps=False,
            imu_hz=10.0,
            lidar_hz=1.0,
            vehicle_hz=10.0,
        )
    )
    seen = []

    def on_tick(t):
        seen.append(t.get('map_id'))

    unsub = hub.subscribe(on_tick)
    hub.start()
    time.sleep(0.4)
    hub.stop()
    unsub()
    assert 'open_pit_bench' in seen
    assert hub.live_tick()['type'] == 'tick'
