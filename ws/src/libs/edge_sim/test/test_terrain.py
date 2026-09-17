"""Haul-road cross-section: width, cut/fill sides, bunds, and lane keeping."""

from __future__ import annotations

import numpy as np

from edge_sim.maps import MAPS, get_map
from edge_sim.models.vehicle import VehicleSimulator
from edge_sim.models.world import HaulWorld


def _section(world: HaulWorld, s: float, laterals: np.ndarray) -> np.ndarray:
    """Terrain height across the road at station ``s``, relative to its centre."""
    cx, cy, yaw = world.centerline(s)
    nx, ny = -np.sin(yaw), np.cos(yaw)
    z = world.height(cx + nx * laterals, cy + ny * laterals)
    return z - float(world.height(cx, cy))


def _high_side_station(world: HaulWorld, want_left: bool) -> float:
    for s in np.arange(40.0, 900.0, 5.0):
        tilt = float(world.hill_tilt(s))
        if want_left and tilt > 0.9:
            return float(s)
        if not want_left and tilt < -0.9:
            return float(s)
    raise AssertionError('no strongly tilted station found')


def test_road_is_wide_enough_for_several_trucks():
    for spec in MAPS.values():
        world = spec.build_world()
        widths = [2.0 * float(world.road_half_width(s)) for s in np.arange(0.0, 600.0, 10.0)]
        # A haul truck is roughly 7 m wide; these roads carry three abreast.
        assert min(widths) > 18.0, spec.id
        assert max(widths) < 45.0, spec.id


def test_road_width_varies_along_the_route():
    world = get_map('haul_corridor').build_world()
    widths = np.array([float(world.road_half_width(s)) for s in np.arange(0.0, 800.0, 5.0)])
    assert np.ptp(widths) > 2.0


def test_one_shoulder_climbs_and_the_other_falls():
    world = get_map('haul_corridor').build_world()
    for want_left in (True, False):
        s = _high_side_station(world, want_left)
        half_width = float(world.road_half_width(s))
        offsets = np.array([half_width + 6.0, half_width + 12.0])
        high = _section(world, s, offsets if want_left else -offsets)
        low = _section(world, s, -offsets if want_left else offsets)
        assert high[0] > 2.0 and high[1] > high[0], 'cut side should climb away'
        assert low[1] < -2.0, 'fill side should fall away'


def test_bund_sits_on_the_falling_shoulder_only():
    world = get_map('haul_corridor').build_world()
    s = _high_side_station(world, want_left=True)
    half_width = float(world.road_half_width(s))
    # Sample right across each shoulder; the berm is a local high point just
    # outside the running surface.
    band = np.arange(0.0, 3.2, 0.1)
    cut = _section(world, s, half_width + band)
    fill = _section(world, s, -(half_width + band))
    assert fill.max() > 0.8, 'falling shoulder needs a bund'
    # The climbing side rises monotonically: no berm crest standing above it.
    assert np.all(np.diff(cut) >= -0.05)


def test_bund_defects_land_on_a_shoulder_that_carries_a_bund():
    for spec in MAPS.values():
        world = spec.build_world()
        assert world.bund_defects, spec.id
        for defect in world.bund_defects:
            tilt = float(world.hill_tilt(defect.s_start + 0.5 * defect.length_m))
            # side > 0 is the left shoulder, which only carries a bund when the
            # hill rises to the right (negative tilt).
            assert (defect.side > 0 and tilt < 0.0) or (defect.side < 0 and tilt > 0.0), (
                f'{spec.id}: {defect.kind} defect on a cut shoulder'
            )


def test_defect_lowers_the_bund_it_sits_on():
    world = get_map('haul_corridor').build_world()
    defect = next(d for d in world.bund_defects if d.kind == 'low')
    mid = defect.s_start + 0.5 * defect.length_m
    band = np.arange(0.0, 3.2, 0.1)
    inside = _section(world, mid, defect.side * (float(world.road_half_width(mid)) + band)).max()
    clear_s = defect.s_start - 45.0
    outside = _section(
        world, clear_s, defect.side * (float(world.road_half_width(clear_s)) + band)
    ).max()
    assert inside < 0.75 * outside


def test_truck_runs_in_a_lane_not_down_the_middle():
    spec = get_map('haul_corridor')
    world = spec.build_world()
    vehicle = VehicleSimulator(speed_mps=spec.speed_mps, world=world, seed=spec.seed + 7)
    offsets = []
    for i in range(120):
        state = vehicle.step(i * 0.2)
        cx, cy, yaw = world.centerline(vehicle._s)
        nx, ny = -np.sin(yaw), np.cos(yaw)
        offsets.append((state.x - cx) * nx + (state.y - cy) * ny)
    offsets = np.asarray(offsets)
    assert np.all(offsets > 2.0), 'truck should hold one side of the road'
    assert np.all(offsets < np.asarray([float(world.road_half_width(0.0))] * len(offsets)))
