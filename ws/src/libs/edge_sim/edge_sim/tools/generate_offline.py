#!/usr/bin/env python3
"""Generate synchronized synthetic sensor CSVs/NPZ without ROS."""

from __future__ import annotations

import argparse
import csv
import json
import math
from contextlib import ExitStack
from pathlib import Path

import numpy as np

from edge_sim.models import GnssSimulator, HaulWorld, ImuSimulator, LidarSimulator, VehicleSimulator


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Offline haul-edge sensor data generator')
    parser.add_argument('--seconds', type=float, default=60.0, help='stream length (default 1 min)')
    parser.add_argument('--imu-hz', type=float, default=100.0)
    parser.add_argument('--gnss-hz', type=float, default=5.0)
    parser.add_argument('--lidar-hz', type=float, default=10.0)
    parser.add_argument('--out', type=str, default='data/sample_run')
    parser.add_argument('--save-lidar-frames', action='store_true')
    args = parser.parse_args(argv)

    for name in ('seconds', 'imu_hz', 'gnss_hz', 'lidar_hz'):
        value = float(getattr(args, name))
        if not math.isfinite(value) or value <= 0:
            parser.error(f'--{name.replace("_", "-")} must be a positive finite value')

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    # This CSV/NPZ export is intentionally not the JSONL replay format.
    # Remove an old ambiguous replay manifest if the directory is reused.
    (out / 'manifest.json').unlink(missing_ok=True)

    world = HaulWorld(seed=19)
    vehicle = VehicleSimulator(world=world)
    imu = ImuSimulator()
    gnss = GnssSimulator()
    lidar = LidarSimulator(world=world)
    vehicle.reset(0.0)

    frames_dir = out / 'lidar_frames'
    if args.save_lidar_frames:
        frames_dir.mkdir(exist_ok=True)

    periods = {
        'imu': 1.0 / args.imu_hz,
        'gnss': 1.0 / args.gnss_hz,
        'lidar': 1.0 / args.lidar_hz,
    }
    next_sample = {kind: 0.0 for kind in periods}
    lidar_meta = []

    with ExitStack() as stack:
        vehicle_file = stack.enter_context((out / 'vehicle.csv').open('w', newline=''))
        imu_file = stack.enter_context((out / 'imu.csv').open('w', newline=''))
        gnss_file = stack.enter_context((out / 'gnss.csv').open('w', newline=''))
        vehicle_writer = csv.writer(vehicle_file)
        imu_writer = csv.writer(imu_file)
        gnss_writer = csv.writer(gnss_file)
        vehicle_writer.writerow(['t', 'x', 'y', 'z', 'yaw', 'pitch', 'roll', 'vx', 'vy', 'vz'])
        imu_writer.writerow(['t', 'gx', 'gy', 'gz', 'ax', 'ay', 'az'])
        gnss_writer.writerow(['t', 'lat', 'lon', 'alt', 'h_acc_m', 'v_acc_m', 'fix_ok', 'status'])

        while True:
            t = min(next_sample.values())
            if t >= args.seconds - 1e-12:
                break
            state = vehicle.step(float(t))
            due = [kind for kind, sample_t in next_sample.items() if sample_t <= t + 1e-9]

            if 'imu' in due:
                vehicle_writer.writerow(
                    [state.t, state.x, state.y, state.z, state.yaw, state.pitch, state.roll,
                     state.vx, state.vy, state.vz]
                )
                sample = imu.sample(state)
                imu_writer.writerow([sample.t, sample.gx, sample.gy, sample.gz, sample.ax, sample.ay, sample.az])

            if 'gnss' in due:
                sample = gnss.sample(state)
                gnss_writer.writerow(
                    [sample.t, sample.latitude_deg, sample.longitude_deg, sample.altitude_m,
                     sample.h_acc_m, sample.v_acc_m, int(sample.fix_ok), sample.status]
                )

            if 'lidar' in due:
                frame = lidar.sample(vehicle, state)
                frame_index = len(lidar_meta)
                lidar_meta.append({'i': frame_index, 't': frame.t, 'n_points': int(frame.points.shape[0])})
                if args.save_lidar_frames:
                    np.savez_compressed(
                        frames_dir / f'frame_{frame_index:05d}.npz',
                        points=frame.points,
                        intensity=frame.intensity,
                        t=np.asarray([frame.t], dtype=np.float64),
                    )

            for kind in due:
                next_sample[kind] += periods[kind]

    (out / 'lidar_meta.json').write_text(json.dumps(lidar_meta, indent=2) + '\n')
    (out / 'offline_manifest.json').write_text(
        json.dumps(
            {
                'format': 'edge-sim-offline-csv-v1',
                'duration_s': args.seconds,
                'imu_hz': args.imu_hz,
                'gnss_hz': args.gnss_hz,
                'lidar_hz': args.lidar_hz,
                'files': ['vehicle.csv', 'imu.csv', 'gnss.csv', 'lidar_meta.json'],
            },
            indent=2,
        )
        + '\n'
    )
    print(f'Wrote offline run to {out.resolve()}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
