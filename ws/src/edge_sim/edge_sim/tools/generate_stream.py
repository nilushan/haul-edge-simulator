#!/usr/bin/env python3
"""Generate replayable multi-map sensor streams under data/streams/."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import List, Optional

from edge_sim.maps import DEFAULT_PLAYLIST, MAPS, list_maps
from edge_sim.stream.format import default_streams_root, discover_streams
from edge_sim.stream.hub import record_map_stream


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(description='Record replayable sensor streams for one or more maps')
    p.add_argument(
        '--maps',
        default=','.join(DEFAULT_PLAYLIST),
        help=f'comma-separated map ids (default: all). Known: {", ".join(MAPS)}',
    )
    p.add_argument('--seconds', type=float, default=60.0)
    p.add_argument('--out-root', type=str, default='', help='default: <repo>/data/streams')
    p.add_argument('--imu-hz', type=float, default=50.0)
    p.add_argument('--gnss-hz', type=float, default=5.0)
    p.add_argument('--lidar-hz', type=float, default=5.0)
    p.add_argument('--vehicle-hz', type=float, default=50.0)
    p.add_argument('--list', action='store_true', help='list maps + existing streams and exit')
    args = p.parse_args(argv)

    out_root = Path(args.out_root) if args.out_root else default_streams_root()

    if args.list:
        print(json.dumps({'maps': list_maps(), 'streams': discover_streams(out_root)}, indent=2))
        return 0

    map_ids = [m.strip() for m in args.maps.split(',') if m.strip()]
    out_root.mkdir(parents=True, exist_ok=True)

    written = []
    for mid in map_ids:
        dest = out_root / mid
        print(f'[generate_stream] recording map={mid} → {dest} ({args.seconds:.0f}s) ...')
        path = record_map_stream(
            mid,
            dest,
            duration_s=args.seconds,
            imu_hz=args.imu_hz,
            gnss_hz=args.gnss_hz,
            lidar_hz=args.lidar_hz,
            vehicle_hz=args.vehicle_hz,
        )
        written.append(str(path.resolve()))
        print(f'[generate_stream] done {mid}')

    index = {
        'streams_root': str(out_root.resolve()),
        'streams': discover_streams(out_root),
        'wrote': written,
    }
    (out_root / 'index.json').write_text(json.dumps(index, indent=2) + '\n')
    print(json.dumps(index, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
