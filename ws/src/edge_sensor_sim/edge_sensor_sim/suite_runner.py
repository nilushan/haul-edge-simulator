"""Real-time multi-rate sensor suite runner with ring buffers (~1 min history)."""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import asdict, dataclass
from typing import Any, Deque, Dict, List, Optional

import numpy as np

from edge_sensor_sim.models import (
    GnssSimulator,
    HaulWorld,
    ImuSimulator,
    LidarSimulator,
    VehicleSimulator,
)


@dataclass
class SuiteConfig:
    duration_s: float = 60.0
    loop: bool = True
    imu_hz: float = 50.0       # slightly lower for web bandwidth
    gnss_hz: float = 5.0
    lidar_hz: float = 5.0      # BEV updates
    vehicle_hz: float = 50.0
    speed_mps: float = 8.0
    history_s: float = 60.0
    lidar_max_points: int = 4000


class SuiteRunner:
    """
    Background thread advances shared vehicle time and samples sensors.
    Keeps ~history_s of samples for REST history + WS live push.
    """

    def __init__(self, cfg: Optional[SuiteConfig] = None) -> None:
        self.cfg = cfg or SuiteConfig()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self.world = HaulWorld(seed=19)
        self.vehicle = VehicleSimulator(speed_mps=self.cfg.speed_mps, world=self.world)
        self.imu = ImuSimulator()
        self.gnss = GnssSimulator()
        self.lidar = LidarSimulator(world=self.world)

        hist = self.cfg.history_s
        self.imu_buf: Deque[Dict[str, Any]] = deque(maxlen=max(10, int(hist * self.cfg.imu_hz) + 5))
        self.gnss_buf: Deque[Dict[str, Any]] = deque(maxlen=max(10, int(hist * self.cfg.gnss_hz) + 5))
        self.odom_buf: Deque[Dict[str, Any]] = deque(maxlen=max(10, int(hist * self.cfg.vehicle_hz) + 5))
        self._last_lidar: Optional[Dict[str, Any]] = None
        self._t = 0.0
        self._cycle = 0
        self._started_wall = 0.0
        self._running = False

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._running = True
        self._started_wall = time.time()
        self.vehicle.reset(0.0)
        self._t = 0.0
        self._thread = threading.Thread(target=self._loop, name='suite-runner', daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        cfg = self.cfg
        next_imu = 0.0
        next_gnss = 0.0
        next_lidar = 0.0
        next_odom = 0.0
        dt_odom = 1.0 / cfg.vehicle_hz
        t0_wall = time.perf_counter()

        while not self._stop.is_set():
            wall = time.perf_counter() - t0_wall
            # map wall time into [0, duration) if looping
            # Continuous world time (never teleport the vehicle). duration_s is only
            # used for history buffer sizing / UI; looping no longer resets pose.
            if cfg.loop:
                self._t = wall
                # cycle counter for UI only (each duration_s window)
                new_cycle = int(wall // max(cfg.duration_s, 1.0))
                if new_cycle != self._cycle:
                    self._cycle = new_cycle
                    # Keep vehicle + map continuity — do NOT reset pose or clear sensors.
            else:
                self._t = min(wall, cfg.duration_s)
                if wall >= cfg.duration_s:
                    self._running = False
                    break

            st = self.vehicle.step(self._t)

            if self._t + 1e-9 >= next_odom:
                next_odom = self._t + dt_odom
                speed = float(np.hypot(st.vx, st.vy))
                odom = {
                    't': st.t,
                    'x': st.x,
                    'y': st.y,
                    'z': st.z,
                    'yaw': st.yaw,
                    'pitch': st.pitch,
                    'roll': st.roll,
                    'vx': st.vx,
                    'vy': st.vy,
                    'vz': st.vz,
                    'yaw_rate': st.yaw_rate,
                    'pitch_rate': st.pitch_rate,
                    'roll_rate': st.roll_rate,
                    'ax': st.ax,
                    'ay': st.ay,
                    'az': st.az,
                    'speed': speed,
                }
                with self._lock:
                    self.odom_buf.append(odom)

            if self._t + 1e-9 >= next_imu:
                next_imu = self._t + 1.0 / cfg.imu_hz
                s = self.imu.sample(st)
                sample = {
                    't': s.t,
                    'gx': s.gx, 'gy': s.gy, 'gz': s.gz,
                    'ax': s.ax, 'ay': s.ay, 'az': s.az,
                }
                with self._lock:
                    self.imu_buf.append(sample)

            if self._t + 1e-9 >= next_gnss:
                next_gnss = self._t + 1.0 / cfg.gnss_hz
                g = self.gnss.sample(st)
                sample = {
                    't': g.t,
                    'lat': g.latitude_deg,
                    'lon': g.longitude_deg,
                    'alt': g.altitude_m,
                    'fix_ok': g.fix_ok,
                    'status': g.status,
                    'x': st.x,
                    'y': st.y,
                }
                with self._lock:
                    self.gnss_buf.append(sample)

            if self._t + 1e-9 >= next_lidar:
                next_lidar = self._t + 1.0 / cfg.lidar_hz
                fr = self.lidar.sample(self.vehicle, st)
                pts = fr.points
                if pts.shape[0] > cfg.lidar_max_points:
                    idx = np.linspace(0, pts.shape[0] - 1, cfg.lidar_max_points).astype(int)
                    pts = pts[idx]
                    inten = fr.intensity[idx]
                else:
                    inten = fr.intensity
                payload = {
                    't': fr.t,
                    'n': int(pts.shape[0]),
                    'xy': np.round(pts[:, :2], 3).tolist(),
                    'z': np.round(pts[:, 2], 3).tolist(),
                    'i': np.round(inten.astype(float), 3).tolist(),
                }
                with self._lock:
                    self._last_lidar = payload

            # Real-time pace (~vehicle rate)
            time.sleep(dt_odom)

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            imu = list(self.imu_buf)
            gnss = list(self.gnss_buf)
            odom = list(self.odom_buf)
            lidar = self._last_lidar
        return {
            'running': self._running and not self._stop.is_set(),
            't': self._t,
            'cycle': self._cycle,
            'duration_s': self.cfg.duration_s,
            'loop': self.cfg.loop,
            'rates': {
                'imu_hz': self.cfg.imu_hz,
                'gnss_hz': self.cfg.gnss_hz,
                'lidar_hz': self.cfg.lidar_hz,
            },
            'latest': {
                'imu': imu[-1] if imu else None,
                'gnss': gnss[-1] if gnss else None,
                'odom': odom[-1] if odom else None,
                'lidar_n': (lidar or {}).get('n'),
            },
            'history': {
                'imu': imu,
                'gnss': gnss,
                'odom': odom,
            },
            'lidar': lidar,
        }

    def live_tick(self) -> Dict[str, Any]:
        """Compact live frame for WebSocket clients."""
        with self._lock:
            imu = self.imu_buf[-1] if self.imu_buf else None
            gnss = self.gnss_buf[-1] if self.gnss_buf else None
            odom = self.odom_buf[-1] if self.odom_buf else None
            lidar = self._last_lidar
            # short tails for sparkline charts
            imu_tail = list(self.imu_buf)[-200:]
            odom_tail = list(self.odom_buf)[-300:]
            gnss_tail = list(self.gnss_buf)[-60:]
        return {
            'type': 'tick',
            't': self._t,
            'cycle': self._cycle,
            'running': self._running and not self._stop.is_set(),
            'imu': imu,
            'gnss': gnss,
            'odom': odom,
            'lidar': lidar,
            'imu_tail': imu_tail,
            'odom_tail': odom_tail,
            'gnss_tail': gnss_tail,
        }
