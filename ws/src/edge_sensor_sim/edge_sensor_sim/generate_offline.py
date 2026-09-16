#!/usr/bin/env python3
"""Generate synthetic sensor CSVs/NPZ without ROS."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from edge_sensor_sim.models import (
    GnssSimulator,
    HaulWorld,
    ImuSimulator,
    LidarSimulator,
    VehicleSimulator,
)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description='Offline haul-edge sensor data generator')
    p.add_argument('--seconds', type=float, default=60.0, help='stream length (default 1 min)')
    p.add_argument('--imu-hz', type=float, default=100.0)
    p.add_argument('--gnss-hz', type=float, default=5.0)
    p.add_argument('--lidar-hz', type=float, default=10.0)
    p.add_argument('--out', type=str, default='data/sample_run')
    p.add_argument('--save-lidar-frames', action='store_true')
    args = p.parse_args(argv)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    world = HaulWorld(seed=19)
    vehicle = VehicleSimulator(world=world)
    imu = ImuSimulator()
    gnss = GnssSimulator()
    lidar = LidarSimulator(world=world)

    # timelines
    t_end = float(args.seconds)
    imu_ts = np.arange(0.0, t_end, 1.0 / args.imu_hz)
    gnss_ts = np.arange(0.0, t_end, 1.0 / args.gnss_hz)
    lidar_ts = np.arange(0.0, t_end, 1.0 / args.lidar_hz)

    vehicle.reset(0.0)

    with (out / 'vehicle.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'x', 'y', 'z', 'yaw', 'pitch', 'roll', 'vx', 'vy', 'vz'])
        # dense vehicle from imu rate
        for t in imu_ts:
            st = vehicle.step(float(t))
            w.writerow([st.t, st.x, st.y, st.z, st.yaw, st.pitch, st.roll, st.vx, st.vy, st.vz])

    vehicle.reset(0.0)
    with (out / 'imu.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'gx', 'gy', 'gz', 'ax', 'ay', 'az'])
        for t in imu_ts:
            st = vehicle.step(float(t))
            s = imu.sample(st)
            w.writerow([s.t, s.gx, s.gy, s.gz, s.ax, s.ay, s.az])

    vehicle.reset(0.0)
    with (out / 'gnss.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'lat', 'lon', 'alt', 'fix_ok', 'status'])
        t_cursor = 0.0
        for t in gnss_ts:
            while t_cursor < t:
                t_cursor = t
            st = vehicle.step(float(t))
            s = gnss.sample(st)
            w.writerow([s.t, s.latitude_deg, s.longitude_deg, s.altitude_m, int(s.fix_ok), s.status])

    vehicle.reset(0.0)
    lidar_meta = []
    frames_dir = out / 'lidar_frames'
    if args.save_lidar_frames:
        frames_dir.mkdir(exist_ok=True)

    for i, t in enumerate(lidar_ts):
        st = vehicle.step(float(t))
        # catch-up: step is absolute time based
        fr = lidar.sample(vehicle, st)
        meta = {'i': i, 't': fr.t, 'n_points': int(fr.points.shape[0])}
        lidar_meta.append(meta)
        if args.save_lidar_frames:
            np.savez_compressed(
                frames_dir / f'frame_{i:05d}.npz',
                points=fr.points,
                intensity=fr.intensity,
                t=np.array([fr.t], dtype=np.float64),
            )

    (out / 'lidar_meta.json').write_text(json.dumps(lidar_meta, indent=2))
    (out / 'manifest.json').write_text(
        json.dumps(
            {
                'seconds': args.seconds,
                'imu_hz': args.imu_hz,
                'gnss_hz': args.gnss_hz,
                'lidar_hz': args.lidar_hz,
                'files': ['vehicle.csv', 'imu.csv', 'gnss.csv', 'lidar_meta.json'],
            },
            indent=2,
        )
    )
    print(f'Wrote offline run to {out.resolve()}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
