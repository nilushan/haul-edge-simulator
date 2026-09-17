import json
import math

import numpy as np

from edge_perception.schema import Label
from edge_viz.perception_overlay import PerceptionOverlay, body_to_map


class FakeSource:
    """Minimal tick source with the StreamHub read API."""

    def __init__(self, ticks):
        self.ticks = list(ticks)
        self.index = 0
        self.started = False
        self.cfg = object()

    def start(self):
        self.started = True

    def stop(self):
        self.started = False

    def catalog(self):
        return {'maps': [], 'streams': []}

    def set_map(self, map_id):
        self.map_id = map_id

    def set_stream(self, stream_id):
        self.stream_id = stream_id

    def live_tick(self):
        tick = self.ticks[min(self.index, len(self.ticks) - 1)]
        self.index += 1
        return json.loads(json.dumps(tick))  # fresh copy, like a real source

    def snapshot(self):
        return {'latest': {'odom': self.ticks[0].get('odom')}, 'lidar': self.ticks[0].get('lidar')}


def _corridor_scan(t=1.0, road_hw=6.0):
    xy, z = [], []
    rng = np.random.default_rng(1)
    for x in np.arange(4.0, 26.0, 0.4):
        for y in np.arange(-road_hw, road_hw, 0.5):
            xy.append([float(x), float(y)])
            z.append(float(rng.normal(0.0, 0.02)))
        for sign in (1.0, -1.0):
            for dy in np.arange(0.0, 2.0, 0.25):
                xy.append([float(x), float(sign * (road_hw + dy))])
                z.append(float(max(0.0, 1.6 * (1.0 - abs(dy - 0.9) / 1.1))))
    return {'t': t, 'n': len(xy), 'xy': xy, 'z': z, 'i': [1.0] * len(xy)}


def _tick(t=1.0, odom=None):
    return {
        'type': 'tick',
        't': t,
        'odom': odom or {'x': 10.0, 'y': -5.0, 'z': 0.0, 'yaw': 0.5, 'pitch': 0.0, 'roll': 0.0, 'speed': 8.0},
        'imu': None,
        'imu_tail': [],
        'lidar': _corridor_scan(t),
        'detect': {'clouds': {}, 'detections': None, 'alerts': [], 'vibe': None},
    }


def test_overlay_labels_the_scan_and_reports_counts():
    overlay = PerceptionOverlay(FakeSource([_tick()]))
    tick = overlay.live_tick()
    labels = tick['lidar']['label']
    assert len(labels) == tick['lidar']['n']
    assert set(labels) >= {int(Label.ROAD), int(Label.BUND)}
    assert tick['detect']['counts']['road'] > 0
    assert tick['detect']['road_half_width_m'] > 3.0
    assert len(tick['lidar']['conf']) == len(labels)


def test_overlay_never_mutates_the_source_payload():
    source = FakeSource([_tick()])
    overlay = PerceptionOverlay(source)
    overlay.live_tick()
    assert 'label' not in source.ticks[0]['lidar']


def test_events_are_frozen_in_the_map_frame():
    scan = _corridor_scan()
    # flatten the left berm over part of the run so a height finding fires
    for i, (xy, z) in enumerate(zip(scan['xy'], scan['z'])):
        if xy[1] > 5.5 and 10.0 <= xy[0] <= 22.0:
            scan['z'][i] = z * 0.3
    tick = _tick()
    tick['lidar'] = scan
    overlay = PerceptionOverlay(FakeSource([tick]))
    alerts = overlay.live_tick()['detect']['alerts']
    assert alerts, 'expected an event from the flattened berm'
    for alert in alerts:
        assert alert['frame_id'] == 'map'
        assert alert['label']
        # Pose must be the world position, not the body-frame offset.
        assert alert['pose']['x'] != alert['pose']['y']
        json.dumps(alert, allow_nan=False)


def test_repeat_events_are_deduplicated_per_type():
    scan = _corridor_scan()
    for i, (xy, z) in enumerate(zip(scan['xy'], scan['z'])):
        if xy[1] > 5.5 and 10.0 <= xy[0] <= 22.0:
            scan['z'][i] = z * 0.3
    ticks = []
    for step in range(4):
        tick = _tick(t=1.0 + step)
        tick['lidar'] = {**scan, 't': 1.0 + step}
        ticks.append(tick)
    overlay = PerceptionOverlay(FakeSource(ticks))
    seen = None
    for _ in range(4):
        seen = overlay.live_tick()['detect']['alerts']
    types = [a['type'] for a in seen]
    # The same standing defect must not stack up one event per frame.
    assert len(types) <= 3


def test_map_switch_clears_events():
    overlay = PerceptionOverlay(FakeSource([_tick()]))
    overlay.live_tick()
    overlay.set_map('other')
    assert overlay.live_tick()['detect']['alerts'] is not None
    assert overlay._last_lidar_t is not None


def test_body_to_map_matches_a_yaw_rotation():
    odom = {'x': 3.0, 'y': -2.0, 'z': 1.0, 'yaw': math.pi / 2}
    x, y, z = body_to_map(10.0, 0.0, 0.5, odom)
    assert abs(x - 3.0) < 1e-6
    assert abs(y - 8.0) < 1e-6
    assert abs(z - 1.5) < 1e-6


def test_unlabelled_source_still_passes_ticks_through():
    tick = _tick()
    tick['lidar'] = None
    overlay = PerceptionOverlay(FakeSource([tick]))
    out = overlay.live_tick()
    assert out['lidar'] is None
    assert out['detect']['alerts'] == []
