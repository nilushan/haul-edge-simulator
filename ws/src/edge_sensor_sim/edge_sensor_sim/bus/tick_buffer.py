"""In-memory multi-rate tick buffer with the same read API as StreamHub (for viz)."""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, List, Optional

from edge_sensor_sim.bus.contract import SensorBusContract, default_bus_contract


@dataclass
class TickBufferConfig:
    history_s: float = 60.0
    imu_hz: float = 50.0
    gnss_hz: float = 5.0
    vehicle_hz: float = 50.0
    source_label: str = 'bus'


class TickBuffer:
    """
    Consumer-side buffer filled by any ingress (ROS bus, future Zenoh, etc.).

    Exposes snapshot() / live_tick() so viz_server can treat hub and bus the same.
    """

    def __init__(
        self,
        cfg: Optional[TickBufferConfig] = None,
        contract: Optional[SensorBusContract] = None,
    ) -> None:
        self.cfg = cfg or TickBufferConfig()
        self.contract = contract or default_bus_contract()
        self._lock = threading.Lock()
        hist = self.cfg.history_s
        self.imu_buf: Deque[Dict[str, Any]] = deque(maxlen=max(10, int(hist * self.cfg.imu_hz) + 5))
        self.gnss_buf: Deque[Dict[str, Any]] = deque(maxlen=max(10, int(hist * self.cfg.gnss_hz) + 5))
        self.odom_buf: Deque[Dict[str, Any]] = deque(maxlen=max(10, int(hist * self.cfg.vehicle_hz) + 5))
        self._last_lidar: Optional[Dict[str, Any]] = None
        self._last_processed_lidar: Optional[Dict[str, Any]] = None
        self._edge_status: Optional[Dict[str, Any]] = None
        self._t = 0.0
        self._t0_wall = time.time()
        self._running = False
        self._source = self.cfg.source_label
        self._map_id = ''
        self._stream_id = ''
        self._cycle = 0
        self._mode = 'bus'

    # --- lifecycle (no background generator; ingress pushes samples) ------

    def start(self) -> None:
        self._running = True
        self._t0_wall = time.time()

    def stop(self) -> None:
        self._running = False

    def set_source_meta(
        self,
        *,
        source: Optional[str] = None,
        map_id: Optional[str] = None,
        stream_id: Optional[str] = None,
        mode: Optional[str] = None,
    ) -> None:
        with self._lock:
            if source is not None:
                self._source = source
            if map_id is not None:
                self._map_id = map_id
            if stream_id is not None:
                self._stream_id = stream_id
            if mode is not None:
                self._mode = mode

    # --- push API (called by bus ingress) ---------------------------------

    def _touch_t(self, t: Optional[float] = None) -> float:
        if t is None:
            t = time.time() - self._t0_wall
        self._t = float(t)
        return self._t

    def push_imu(self, sample: Dict[str, Any]) -> None:
        with self._lock:
            self._touch_t(sample.get('t'))
            self.imu_buf.append(sample)

    def push_gnss(self, sample: Dict[str, Any]) -> None:
        with self._lock:
            self._touch_t(sample.get('t'))
            self.gnss_buf.append(sample)

    def push_odom(self, sample: Dict[str, Any]) -> None:
        with self._lock:
            self._touch_t(sample.get('t'))
            self.odom_buf.append(sample)

    def push_lidar(self, sample: Dict[str, Any], *, processed: bool = False) -> None:
        with self._lock:
            self._touch_t(sample.get('t'))
            if processed:
                self._last_processed_lidar = sample
            else:
                self._last_lidar = sample

    def push_edge_status(self, status: Dict[str, Any]) -> None:
        with self._lock:
            self._edge_status = status

    # --- read API (viz subscriber) ----------------------------------------

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            imu = list(self.imu_buf)
            gnss = list(self.gnss_buf)
            odom = list(self.odom_buf)
            lidar = self._last_lidar
            processed = self._last_processed_lidar
            status = self._edge_status
            # Prefer processed cloud for display when present
            display_lidar = processed or lidar
        return {
            'running': self._running,
            't': self._t,
            'cycle': self._cycle,
            'duration_s': self.cfg.history_s,
            'loop': True,
            'mode': self._mode,
            'map_id': self._map_id,
            'stream_id': self._stream_id,
            'source': self._source,
            'bus': self.contract.to_dict(),
            'rates': {
                'imu_hz': self.cfg.imu_hz,
                'gnss_hz': self.cfg.gnss_hz,
                'lidar_hz': 0.0,
            },
            'latest': {
                'imu': imu[-1] if imu else None,
                'gnss': gnss[-1] if gnss else None,
                'odom': odom[-1] if odom else None,
                'lidar_n': (display_lidar or {}).get('n'),
                'edge_status': status,
            },
            'history': {'imu': imu, 'gnss': gnss, 'odom': odom},
            'lidar': display_lidar,
            'lidar_raw': lidar,
            'lidar_processed': processed,
        }

    def live_tick(self) -> Dict[str, Any]:
        with self._lock:
            imu = self.imu_buf[-1] if self.imu_buf else None
            gnss = self.gnss_buf[-1] if self.gnss_buf else None
            odom = self.odom_buf[-1] if self.odom_buf else None
            lidar = self._last_processed_lidar or self._last_lidar
            imu_tail = list(self.imu_buf)[-200:]
            odom_tail = list(self.odom_buf)[-300:]
            gnss_tail = list(self.gnss_buf)[-60:]
            status = self._edge_status
        return {
            'type': 'tick',
            't': self._t,
            'cycle': self._cycle,
            'running': self._running,
            'mode': self._mode,
            'map_id': self._map_id,
            'stream_id': self._stream_id,
            'source': self._source,
            'imu': imu,
            'gnss': gnss,
            'odom': odom,
            'lidar': lidar,
            'imu_tail': imu_tail,
            'odom_tail': odom_tail,
            'gnss_tail': gnss_tail,
            'edge_status': status,
        }

    def catalog(self) -> Dict[str, Any]:
        return {
            'mode': self._mode,
            'maps': [],
            'streams': [],
            'bus': self.contract.to_dict(),
            'active': {
                'map_id': self._map_id,
                'stream_id': self._stream_id,
                'source': self._source,
                'cycle': self._cycle,
            },
            'playlist': {'maps': [], 'streams': [], 'cycle': False},
        }

    # Minimal stubs so viz select endpoints fail softly
    def set_map(self, map_id: str) -> None:
        raise RuntimeError('map select only available in direct StreamHub mode, not bus mode')

    def set_stream(self, stream_id: str) -> None:
        raise RuntimeError('stream select only available in direct StreamHub mode, not bus mode')
