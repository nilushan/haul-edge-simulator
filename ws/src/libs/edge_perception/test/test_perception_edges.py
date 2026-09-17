"""Regression tests for edge cases in the pure perception library."""

from __future__ import annotations

import struct
from types import SimpleNamespace

import numpy as np
import pytest

from edge_perception.bunds import BundParams, detect_bunds, extract_crest
from edge_perception.cloud_io import decode_xyz
from edge_perception.ground import estimate_ground_grid
from edge_perception.rocks import detect_rocks
from edge_perception.schema import AlertEvent
from edge_perception.vibration import ImuSample, VibeFeatures, VibeParams, VibrationMonitor


FLOAT32 = 7


def _field(name: str, offset: int) -> SimpleNamespace:
    return SimpleNamespace(name=name, offset=offset, datatype=FLOAT32, count=1)


def test_decode_xyz_supports_big_endian_organized_cloud_with_row_padding():
    width, height, point_step, row_step = 2, 2, 16, 40
    data = bytearray(row_step * height)
    expected = []
    for row in range(height):
        for col in range(width):
            xyz = (float(row * 10 + col), float(row), float(-col))
            expected.append(xyz)
            struct.pack_into('>fff', data, row * row_step + col * point_step + 4, *xyz)
    msg = SimpleNamespace(
        fields=[_field('x', 4), _field('y', 8), _field('z', 12)],
        point_step=point_step,
        row_step=row_step,
        width=width,
        height=height,
        is_bigendian=True,
        data=bytes(data),
    )

    points, intensity = decode_xyz(msg)

    np.testing.assert_allclose(points, np.asarray(expected, dtype=np.float32))
    assert intensity is None


def test_decode_xyz_ignores_truncated_partial_point():
    msg = SimpleNamespace(
        fields=[_field('x', 0), _field('y', 4), _field('z', 8)],
        point_step=16,
        row_step=16,
        width=1,
        height=1,
        is_bigendian=False,
        data=bytes(12),
    )
    points, intensity = decode_xyz(msg)
    assert points.shape == (0, 3)
    assert intensity is None


def test_decode_xyz_returns_immediately_for_zero_width_cloud():
    msg = SimpleNamespace(
        fields=[_field('x', 0), _field('y', 4), _field('z', 8)],
        point_step=12,
        row_step=0,
        width=0,
        height=10**9,
        is_bigendian=False,
        data=b'',
    )
    points, _ = decode_xyz(msg)
    assert points.shape == (0, 3)


def test_decode_xyz_rejects_field_outside_point_step():
    msg = SimpleNamespace(
        fields=[_field('x', 0), _field('y', 4), _field('z', 12)],
        point_step=12,
        row_step=12,
        width=1,
        height=1,
        is_bigendian=False,
        data=bytes(16),
    )
    with pytest.raises(ValueError, match='offsets exceed point_step'):
        decode_xyz(msg)


def test_ground_grid_isolates_non_finite_points():
    points = np.asarray(
        [[0.0, 0.0, 0.0], [0.1, 0.1, 0.05], [0.2, 0.2, np.nan]],
        dtype=np.float32,
    )

    result = estimate_ground_grid(points, cell_m=1.0, min_cell_points=2)

    assert set(result.ground_idx.tolist()) == {0, 1}
    assert np.isnan(result.hag[2])


def test_ground_grid_rejects_non_positive_cell_size():
    with pytest.raises(ValueError, match='cell_m'):
        estimate_ground_grid(np.zeros((1, 3), dtype=np.float32), cell_m=0.0)


def test_alert_event_preserves_explicit_zero_ros_timestamp():
    assert AlertEvent(type='test', t_ros=0.0).to_dict()['t_ros'] == 0.0


def test_detect_rocks_without_hag_is_translation_invariant_in_z():
    rng = np.random.default_rng(4)
    cluster = np.column_stack(
        [
            12.0 + rng.normal(0.0, 0.18, 40),
            1.0 + rng.normal(0.0, 0.18, 40),
            0.5 + rng.normal(0.0, 0.2, 40),
        ]
    ).astype(np.float32)

    low = detect_rocks(cluster)
    high = detect_rocks(cluster + np.asarray([0.0, 0.0, 20.0], dtype=np.float32))

    assert len(low) == len(high) == 1
    assert low[0].radius_m == pytest.approx(high[0].radius_m)


def test_extract_crest_rejects_unknown_side():
    with pytest.raises(ValueError, match='side'):
        extract_crest(np.zeros((0, 3), dtype=np.float32), 'centre')


def test_bund_detection_uses_road_ground_for_height():
    xs = np.linspace(5.0, 30.0, 80)
    road = np.column_stack(
        [
            np.repeat(xs, 3),
            np.tile(np.asarray([-3.0, 0.0, 3.0]), 80),
            np.zeros(240),
        ]
    )
    left_bund = np.column_stack(
        [
            np.repeat(xs, 4),
            np.tile(np.linspace(5.5, 8.0, 4), 80),
            np.tile(np.linspace(0.5, 1.5, 4), 80),
        ]
    )
    points = np.vstack([road, left_bund]).astype(np.float32)

    detections, alerts = detect_bunds(points, BundParams(road_half_width_m=6.5))

    left = next(d for d in detections if d.details['side'] == 'left')
    assert left.details['mean_height_m'] >= 1.0
    assert left.extent_m[2] >= 1.0
    assert any(a['type'] == 'bund_low' and a['side'] == 'right' for a in alerts)


def test_empty_cloud_does_not_claim_a_physical_bund_gap():
    detections, alerts = detect_bunds(
        np.zeros((0, 3), dtype=np.float32), BundParams(road_half_width_m=6.5)
    )
    assert detections == []
    assert alerts == []


def test_one_observed_shoulder_does_not_claim_unobserved_opposite_gap():
    xs = np.linspace(5.0, 30.0, 40)
    left_only = np.column_stack([xs, np.full_like(xs, 7.0), np.ones_like(xs)])
    _, alerts = detect_bunds(left_only.astype(np.float32), BundParams(road_half_width_m=6.5))
    assert not any(a['type'] == 'bund_gap' and a['side'] == 'right' for a in alerts)


def test_vibration_gap_resets_alert_debounce():
    params = VibeParams(hold_s=0.5, min_samples=2, max_gap_s=0.25)
    monitor = VibrationMonitor(params)
    monitor.push(ImuSample(t=1.0, ax=0.0, ay=0.0, az=9.8))
    high = VibeFeatures(
        t=1.0, rms_az=3.0, peak_az=7.0, rms_horiz=0.0,
        peak_horiz=0.0, crest_az=2.0, n=20,
    )
    assert monitor.evaluate(high, speed_mps=5.0) is None
    assert monitor._above_since == 1.0

    monitor.push(ImuSample(t=2.0, ax=0.0, ay=0.0, az=9.8))
    after_gap = VibeFeatures(
        t=2.0, rms_az=3.0, peak_az=7.0, rms_horiz=0.0,
        peak_horiz=0.0, crest_az=2.0, n=20,
    )
    assert monitor.evaluate(after_gap, speed_mps=5.0) is None
    assert monitor._above_since == 2.0


def test_vibration_monitor_resets_window_on_backward_timestamp():
    monitor = VibrationMonitor()
    feature = None
    for i in range(60):
        feature = monitor.push(ImuSample(t=i * 0.02, ax=0.0, ay=0.0, az=9.8)) or feature
    assert feature is not None

    # A replay/clock reset must not mix samples from the old future timeline.
    assert monitor.push(ImuSample(t=0.0, ax=0.0, ay=0.0, az=9.8)) is None
    assert len(monitor._buf) == 1
