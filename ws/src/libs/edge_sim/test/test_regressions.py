"""Regression tests for simulation geometry, timing, and lifecycle edge cases."""

from __future__ import annotations

import math
import time

import numpy as np
import pytest

from edge_sim.maps import MAPS
from edge_sim.models import GnssSimulator, LidarSimulator, VehicleSimulator, VehicleState
from edge_sim.models.world import HaulWorld
from edge_sim.stream.format import StreamManifest, StreamReader, StreamWriter
from edge_sim.stream.hub import StreamConfig, StreamHub, record_map_stream


def test_world_height_supports_numpy_broadcasting():
    world = HaulWorld(seed=1, n_rocks=0)
    result = world.height(10.0, np.asarray([-2.0, 0.0, 2.0]))
    assert result.shape == (3,)
    assert np.all(np.isfinite(result))


def test_vehicle_reported_velocity_matches_position_delta_on_every_map():
    for spec in MAPS.values():
        vehicle = VehicleSimulator(speed_mps=spec.speed_mps, world=spec.build_world())
        first = vehicle.step(0.0)
        second = vehicle.step(0.1)
        position_speed = math.hypot(second.x - first.x, second.y - first.y) / 0.1
        reported_speed = math.hypot(second.vx, second.vy)
        assert position_speed == pytest.approx(spec.speed_mps, rel=0.03)
        assert reported_speed == pytest.approx(position_speed, rel=0.01)


def test_vehicle_reset_replays_random_schedule_deterministically():
    vehicle = VehicleSimulator(seed=42)
    vehicle.reset(0.0)
    first = vehicle.step(20.0)
    vehicle.reset(0.0)
    second = vehicle.step(20.0)
    assert second == first


def test_vehicle_reset_offsets_random_deadlines_from_nonzero_time():
    vehicle = VehicleSimulator(seed=42)
    vehicle.reset(100.0)
    assert vehicle._next_speed_s > 100.0
    assert vehicle._next_bump_s > 100.0


def test_initial_gnss_dropout_does_not_leak_moving_ground_truth():
    vehicle = VehicleSimulator()
    gnss = GnssSimulator(dropout_prob=1.0)
    first = gnss.sample(vehicle.step(1.0))
    second = gnss.sample(vehicle.step(2.0))
    assert not first.fix_ok and not second.fix_ok
    assert (second.latitude_deg, second.longitude_deg, second.altitude_m) == (
        first.latitude_deg, first.longitude_deg, first.altitude_m
    )


def test_gnss_dropout_holds_last_fix_instead_of_teleporting_to_origin():
    vehicle = VehicleSimulator()
    state = vehicle.step(2.0)
    gnss = GnssSimulator(dropout_prob=0.0, h_noise_m=0.0, v_noise_m=0.0)
    valid = gnss.sample(state)
    gnss.dropout_prob = 1.0
    dropped = gnss.sample(vehicle.step(3.0))
    assert not dropped.fix_ok
    assert dropped.latitude_deg == valid.latitude_deg
    assert dropped.longitude_deg == valid.longitude_deg
    assert dropped.altitude_m == valid.altitude_m


def test_lidar_returns_first_narrow_surface_intersection():
    class NarrowBumpWorld:
        @staticmethod
        def height(x, y):
            x = np.asarray(x)
            return np.where((x >= 5.0) & (x <= 6.0), 2.0, 0.0)

    state = VehicleState(
        t=0.0, x=0.0, y=0.0, z=0.0, yaw=0.0, pitch=0.0, roll=0.0,
        vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0, pitch_rate=0.0,
        roll_rate=0.0, ax=0.0, ay=0.0, az=0.0,
    )
    vehicle = VehicleSimulator()
    lidar = LidarSimulator(
        n_rings=1,
        n_azimuth=1,
        min_range_m=1.0,
        max_range_m=30.0,
        noise_std_m=0.0,
        dropout_prob=0.0,
        world=NarrowBumpWorld(),
        mount_xyz=(0.0, 0.0, 1.0),
    )
    direction = np.asarray([[1.0, 0.0, -0.02]])
    lidar._dir = direction / np.linalg.norm(direction, axis=1, keepdims=True)
    lidar._n = 1

    frame = lidar.sample(vehicle, state)

    assert frame.points.shape == (1, 3)
    assert 4.9 <= frame.points[0, 0] <= 5.1


def test_lidar_rejects_unbounded_march_configuration():
    with pytest.raises(ValueError, match='2048 march steps'):
        LidarSimulator(min_range_m=0.0, max_range_m=100.0, ray_step_m=0.001)


def test_stream_config_rejects_invalid_rates_and_duration():
    with pytest.raises(ValueError, match='duration_s'):
        StreamConfig(duration_s=0.0)
    with pytest.raises(ValueError, match='imu_hz'):
        StreamConfig(imu_hz=float('nan'))


def test_stream_reader_sorts_samples_and_includes_gnss_tail(tmp_path):
    root = tmp_path / 'stream'
    manifest = StreamManifest(stream_id='ordered', map_id='haul_corridor', duration_s=1.0)
    with StreamWriter(root, manifest) as writer:
        writer.write('gnss', {'t': 3.0, 'lat': 0.0})
        writer.write('gnss', {'t': 2.0, 'lat': 0.0})
    reader = StreamReader(root)
    assert [sample['t'] for sample in reader.gnss] == [2.0, 3.0]
    assert reader.duration_s == 3.0


def test_manifest_rejects_incompatible_version():
    with pytest.raises(ValueError, match='unsupported stream version'):
        StreamManifest.from_dict({'version': 999})


def test_record_map_stream_honors_rates_above_vehicle_rate(tmp_path):
    root = tmp_path / 'recorded'
    record_map_stream(
        'haul_corridor',
        root,
        duration_s=0.5,
        imu_hz=100.0,
        gnss_hz=7.0,
        lidar_hz=1.0,
        vehicle_hz=20.0,
        lidar_max_points=100,
    )
    reader = StreamReader(root)
    assert len(reader.imu) == 51
    assert len(reader.odom) == 11
    assert len(reader.gnss) == 4
    assert len(reader.lidar) == 1
    np.testing.assert_allclose(np.diff([s['t'] for s in reader.imu]), 0.01, atol=1e-9)


def test_running_hub_can_switch_from_live_to_replay(tmp_path):
    root = tmp_path / 'switch-stream'
    manifest = StreamManifest(stream_id='switch-stream', map_id='rocky_descent', duration_s=0.3)
    with StreamWriter(root, manifest) as writer:
        writer.write('odom', {'t': 0.0, 'x': 123.0})
        writer.write('odom', {'t': 0.1, 'x': 124.0})

    hub = StreamHub(
        StreamConfig(
            mode='live', map_ids=['haul_corridor'], duration_s=2.0,
            imu_hz=10.0, gnss_hz=2.0, lidar_hz=1.0, vehicle_hz=20.0,
        )
    )
    hub.start()
    time.sleep(0.08)
    hub.set_stream(str(root))
    time.sleep(0.15)
    tick = hub.live_tick()
    hub.stop()

    assert tick['mode'] == 'replay'
    assert tick['stream_id'] == 'switch-stream'
    assert tick['map_id'] == 'rocky_descent'
    assert tick['odom']['x'] in {123.0, 124.0}


def test_replay_restart_resets_cursors_and_history(tmp_path):
    root = tmp_path / 'restart-stream'
    with StreamWriter(root, StreamManifest(stream_id='restart', duration_s=0.4)) as writer:
        writer.write('odom', {'t': 0.0, 'x': 1.0})
        writer.write('odom', {'t': 0.2, 'x': 2.0})

    hub = StreamHub(StreamConfig(mode='replay', stream_path=str(root), vehicle_hz=50.0))
    hub.start()
    time.sleep(0.25)
    hub.stop()
    assert hub.live_tick()['odom']['x'] == 2.0

    hub.start()
    time.sleep(0.05)
    restarted = hub.snapshot()
    hub.stop()
    assert restarted['history']['odom'][0]['t'] == 0.0
    assert restarted['latest']['odom']['x'] == 1.0
