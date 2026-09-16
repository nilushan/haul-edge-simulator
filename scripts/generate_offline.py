#!/usr/bin/env python3
"""Host entrypoint for offline generation (no ROS required)."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'ws' / 'src' / 'edge_sensor_sim'
sys.path.insert(0, str(PKG))

from edge_sensor_sim.adapters.files.generate_offline import main  # noqa: E402

if __name__ == '__main__':
    raise SystemExit(main())
