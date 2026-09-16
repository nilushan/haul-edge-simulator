"""Backward-compatible facade: SuiteRunner is now a thin alias over StreamHub.

The hub is the sole sensor-data generator. Prefer importing StreamHub directly.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

from edge_sensor_sim.maps import DEFAULT_PLAYLIST
from edge_sensor_sim.stream.hub import StreamConfig, StreamHub


@dataclass
class SuiteConfig:
    duration_s: float = 60.0
    loop: bool = True
    imu_hz: float = 50.0
    gnss_hz: float = 5.0
    lidar_hz: float = 5.0
    vehicle_hz: float = 50.0
    speed_mps: float = 8.0
    history_s: float = 60.0
    lidar_max_points: int = 4000
    mode: str = 'live'  # live | replay
    map_ids: Optional[Sequence[str]] = None
    stream_path: Optional[str] = None
    streams_root: Optional[str] = None
    cycle_maps: bool = True


class SuiteRunner(StreamHub):
    """Alias kept so older imports keep working."""

    def __init__(self, cfg: Optional[SuiteConfig] = None) -> None:
        sc = cfg or SuiteConfig()
        hub_cfg = StreamConfig(
            mode=sc.mode,
            map_ids=list(sc.map_ids) if sc.map_ids else list(DEFAULT_PLAYLIST),
            stream_path=sc.stream_path,
            streams_root=sc.streams_root,
            duration_s=sc.duration_s,
            loop=sc.loop,
            cycle_maps=sc.cycle_maps,
            imu_hz=sc.imu_hz,
            gnss_hz=sc.gnss_hz,
            lidar_hz=sc.lidar_hz,
            vehicle_hz=sc.vehicle_hz,
            history_s=sc.history_s,
            lidar_max_points=sc.lidar_max_points,
            speed_mps=sc.speed_mps,
        )
        super().__init__(hub_cfg)
