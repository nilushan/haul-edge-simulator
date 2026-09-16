"""Unit tests for pure domain sensor models (no ROS)."""

from __future__ import annotations

import math

import numpy as np

from edge_sim.models import (
    GnssSimulator,
    HaulWorld,
    ImuSimulator,
    LidarSimulator,
    VehicleSimulator,
)


def test_vehicle_moves_forward():
    w = HaulWorld(seed=1)
    v = VehicleSimulator(speed_mps=10.0, world=w)
    v.reset(0.0)
    s0 = v.step(0.0)
    s1 = v.step(1.0)
    assert math.hypot(s1.x - s0.x, s1.y - s0.y) > 3.0
    assert math.hypot(s1.vx, s1.vy) > 5.0


def test_imu_has_gravity_ish():
    v = VehicleSimulator()
    imu = ImuSimulator(accel_noise_std=0.0, gyro_noise_std=0.0)
    v.reset(0.0)
    st = v.step(0.0)
    s = imu.sample(st)
    assert s.az > 8.0


def test_gnss_fix_fields():
    v = VehicleSimulator()
    g = GnssSimulator(dropout_prob=0.0)
    st = v.step(0.0)
    s = g.sample(st)
    assert s.fix_ok
    assert -90.0 <= s.latitude_deg <= 90.0


def test_lidar_returns_points():
    w = HaulWorld(seed=2)
    v = VehicleSimulator(world=w)
    lid = LidarSimulator(world=w, n_rings=8, n_azimuth=90, ray_iters=10)
    st = v.step(0.0)
    fr = lid.sample(v, st)
    assert fr.points.ndim == 2 and fr.points.shape[1] == 3
    assert fr.points.shape[0] > 30
    assert fr.intensity.shape[0] == fr.points.shape[0]
    rng = np.linalg.norm(fr.points, axis=1)
    assert rng.std() > 0.5
