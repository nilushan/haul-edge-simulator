import json

import numpy as np

from edge_perception.schema import Label, class_style, style_catalog
from edge_perception.semantics import (
    SemanticParams,
    classify_frame,
    estimate_road_half_width,
)


def _corridor(
    *,
    road_hw=6.0,
    berm_h=1.6,
    x_range=(4.0, 28.0),
    berm_sides=('left', 'right'),
    low_side=None,
    low_height=0.5,
    gap_side=None,
    gap_x=(14.0, 26.0),
    seed=0,
):
    """Synthetic haul corridor: road plane, both berms, optional defects."""
    rng = np.random.default_rng(seed)
    xs = np.arange(x_range[0], x_range[1], 0.35)
    pts = []
    for x in xs:
        for y in np.arange(-road_hw, road_hw, 0.4):
            pts.append((x, y, rng.normal(0.0, 0.02)))
        for side in berm_sides:
            sign = 1.0 if side == 'left' else -1.0
            if gap_side == side and gap_x[0] <= x <= gap_x[1]:
                continue
            height = berm_h
            if low_side == side and gap_x[0] <= x <= gap_x[1]:
                height = low_height
            for dy in np.arange(0.0, 2.0, 0.25):
                y = sign * (road_hw + dy)
                z = height * (1.0 - abs(dy - 0.9) / 1.1)
                pts.append((x, y, max(0.0, z) + rng.normal(0.0, 0.02)))
    return np.asarray(pts, dtype=np.float64)


def test_classifies_road_ground_and_bund():
    pts = _corridor()
    res = classify_frame(pts)
    assert res.counts.get('road', 0) > 100
    assert res.counts.get('bund', 0) > 50
    assert res.counts.get('bund_low', 0) == 0
    assert res.labels.shape[0] == pts.shape[0]
    # Points on the lane centre are road, points on the berm crest are bund.
    centre = (np.abs(pts[:, 1]) < 2.0) & np.isfinite(res.hag)
    assert np.all(res.labels[centre] == int(Label.ROAD))
    crest = pts[:, 2] > 1.2
    assert np.all(res.labels[crest] == int(Label.BUND))


def test_road_half_width_tracks_the_berm_toe():
    for road_hw in (4.5, 6.0, 9.0):
        pts = _corridor(road_hw=road_hw)
        assert abs(estimate_road_half_width(pts, SemanticParams()) - road_hw) <= 1.0


def test_low_bund_section_is_labelled_and_reported():
    pts = _corridor(low_side='left', low_height=0.55)
    res = classify_frame(pts)
    assert res.counts.get('bund_low', 0) > 0
    low = [e for e in res.events if e.type == 'bund_low']
    assert low, 'expected a bund height finding'
    event = low[0]
    assert event.details['side'] == 'left'
    assert event.details['min_height_m'] < event.details['min_required_m']
    assert event.details['deficit_m'] > 0
    assert 'BUND LOW' in event.label
    assert event.y > 0  # left of the vehicle


def test_missing_bund_section_reports_a_gap():
    pts = _corridor(gap_side='right', gap_x=(8.0, 24.0))
    res = classify_frame(pts)
    gaps = [e for e in res.events if e.type == 'bund_gap']
    assert gaps, 'expected a bund gap finding'
    assert gaps[0].details['side'] == 'right'
    assert gaps[0].details['length_m'] >= gaps[0].details['gap_length_m']


def test_compliant_corridor_raises_no_bund_findings():
    res = classify_frame(_corridor(berm_h=1.8))
    assert [e.type for e in res.events if e.type.startswith('bund')] == []


def test_rock_on_the_road_is_labelled_and_reported():
    rng = np.random.default_rng(3)
    pts = list(map(tuple, _corridor()))
    for _ in range(90):
        # dome of returns on a ~0.8 m rock at 14 m, left of centre
        pts.append(
            (
                14.0 + rng.normal(0.0, 0.18),
                1.5 + rng.normal(0.0, 0.18),
                0.55 + rng.normal(0.0, 0.08),
            )
        )
    res = classify_frame(np.asarray(pts, dtype=np.float64))
    assert res.counts.get('rock', 0) > 0
    rocks = [e for e in res.events if e.type == 'rock']
    assert rocks, 'expected a rock event'
    assert abs(rocks[0].x - 14.0) < 1.5
    assert rocks[0].details['in_lane'] is True


def test_unmeasurable_crest_makes_no_height_claim():
    """A shoulder sampled only on its inner flank must not report a low bund."""
    pts = _corridor(x_range=(4.0, 20.0))
    inner_only = pts[(np.abs(pts[:, 1]) <= 6.6)]
    res = classify_frame(inner_only)
    assert [e.type for e in res.events if e.type == 'bund_low'] == []


def test_empty_and_degenerate_frames():
    res = classify_frame(np.zeros((0, 3)))
    assert res.labels.size == 0 and res.events == []
    flat = np.column_stack(
        [np.linspace(5, 20, 200), np.zeros(200), np.zeros(200)]
    )
    assert classify_frame(flat).labels.shape[0] == 200


def test_payloads_are_strict_json():
    pts = _corridor(low_side='left', gap_side='right')
    res = classify_frame(pts)
    for event in res.events:
        json.dumps(event.to_dict(), allow_nan=False)
    for detection in res.detections:
        json.dumps(detection.to_dict(), allow_nan=False)


def test_style_catalog_covers_every_emitted_class():
    catalog = style_catalog()
    keys = {c['key'] for c in catalog['classes']}
    layers = {layer['key'] for layer in catalog['layers']}
    assert {'road', 'ground', 'bund', 'bund_low', 'rock', 'obstacle'} <= keys
    assert {c['layer'] for c in catalog['classes']} <= layers
    for label in Label:
        assert class_style(int(label)).key in keys
    types = {e['type'] for e in catalog['events']}
    assert {'rock', 'bund_low', 'bund_gap', 'excessive_vibration'} <= types


def _asymmetric_corridor(
    *,
    left_edge=7.0,
    right_edge=19.0,
    berm_h=1.6,
    cut_side=None,
    seed=5,
):
    """
    Scan of a truck running in a lane on a wide haul road.

    ``cut_side`` climbs away from the road instead of carrying a bund, which is
    what a road cut into a hillside actually looks like.
    """
    rng = np.random.default_rng(seed)
    pts = []
    for x in np.arange(4.0, 28.0, 0.35):
        for y in np.arange(-right_edge, left_edge, 0.4):
            pts.append((x, y, rng.normal(0.0, 0.02)))
        for side, edge in (('left', left_edge), ('right', right_edge)):
            sign = 1.0 if side == 'left' else -1.0
            if side == cut_side:
                for d in np.arange(0.0, 9.0, 0.3):
                    pts.append((x, sign * (edge + d), 0.75 * d + rng.normal(0.0, 0.03)))
                continue
            for d in np.arange(0.0, 2.6, 0.2):
                z = berm_h * (1.0 - abs(d - 1.2) / 1.4)
                pts.append((x, sign * (edge + d), max(0.0, z) + rng.normal(0.0, 0.02)))
    return np.asarray(pts, dtype=np.float64)


def test_corridor_edges_are_measured_per_side():
    from edge_perception.semantics import estimate_corridor

    corridor = estimate_corridor(_asymmetric_corridor(), SemanticParams())
    assert abs(corridor.left_m - 7.0) <= 1.5
    assert abs(corridor.right_m - 19.0) <= 1.5
    assert corridor.width_m > 20.0


def test_wide_road_is_classified_across_its_whole_width():
    res = classify_frame(_asymmetric_corridor())
    assert res.counts.get('road', 0) > 500
    # A point two thirds of the way to the far shoulder is still road.
    assert res.counts['road'] > res.counts.get('ground', 0)


def test_climbing_shoulder_is_a_cut_batter_not_a_missing_bund():
    res = classify_frame(_asymmetric_corridor(cut_side='left'))
    kinds = {profile.side: profile.kind for profile in res.sides}
    assert kinds['left'] == 'cut'
    assert kinds['right'] == 'fill'
    assert res.counts.get('cut_slope', 0) > 50
    # Nothing has to stop a truck falling uphill, so no finding on that side.
    assert [e for e in res.events if e.details.get('side') == 'left'] == []


def test_low_bund_on_the_near_shoulder_is_still_reported():
    res = classify_frame(_asymmetric_corridor(cut_side='right', berm_h=0.6))
    lows = [e for e in res.events if e.type == 'bund_low']
    assert lows, 'a flattened near bund must still raise a finding'
    assert lows[0].details['side'] == 'left'
