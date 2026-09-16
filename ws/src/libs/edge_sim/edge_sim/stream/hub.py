"""Sole sensor-stream generator: live multi-map synthesis or disk replay."""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Deque, Dict, List, Optional, Sequence

import numpy as np

from edge_sim.maps import (
    DEFAULT_MAP_ID,
    DEFAULT_PLAYLIST,
    MapSpec,
    get_map,
    list_maps,
    resolve_playlist,
)
from edge_sim.models import (
    GnssSimulator,
    ImuSimulator,
    LidarSimulator,
    VehicleSimulator,
)
from edge_sim.stream.format import (
    StreamManifest,
    StreamReader,
    StreamWriter,
    default_streams_root,
    discover_streams,
)


Subscriber = Callable[[Dict[str, Any]], None]


@dataclass
class StreamConfig:
    """Runtime config for the sole stream hub."""

    mode: str = 'live'  # live | replay
    map_ids: Sequence[str] = field(default_factory=lambda: list(DEFAULT_PLAYLIST))
    stream_path: Optional[str] = None  # single stream dir (replay)
    streams_root: Optional[str] = None  # catalog root for multi-stream replay
    stream_ids: Sequence[str] = field(default_factory=list)  # playlist of recorded streams
    duration_s: float = 600.0  # ~10 min per map segment before loop
    loop: bool = True
    cycle_maps: bool = False  # advance map/stream each duration window when enabled
    imu_hz: float = 50.0
    gnss_hz: float = 5.0
    lidar_hz: float = 10.0
    vehicle_hz: float = 50.0
    history_s: float = 60.0
    lidar_max_points: int = 20000
    speed_mps: Optional[float] = None  # None → use map default
    record_dir: Optional[str] = None  # if set in live mode, record first cycle


class StreamHub:
    """
    Single source of truth for sensor samples.

    - live: synthesize from map playlist via pure models
    - replay: play recorded stream directories (JSONL)
    Visualizer / ROS bridges only subscribe — they never own models.
    """

    def __init__(self, cfg: Optional[StreamConfig] = None) -> None:
        self.cfg = cfg or StreamConfig()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._subscribers: List[Subscriber] = []

        hist = self.cfg.history_s
        self.imu_buf: Deque[Dict[str, Any]] = deque(maxlen=max(10, int(hist * self.cfg.imu_hz) + 5))
        self.gnss_buf: Deque[Dict[str, Any]] = deque(maxlen=max(10, int(hist * self.cfg.gnss_hz) + 5))
        self.odom_buf: Deque[Dict[str, Any]] = deque(maxlen=max(10, int(hist * self.cfg.vehicle_hz) + 5))
        self._last_lidar: Optional[Dict[str, Any]] = None

        self._t = 0.0
        self._cycle = 0
        self._running = False
        self._map_index = 0
        self._stream_index = 0
        self._active_map_id = DEFAULT_MAP_ID
        self._active_stream_id = ''
        self._source_label = ''
        self._segment_epoch = 0  # bumped on manual map/stream switch

        self._maps: List[MapSpec] = []
        self._readers: List[StreamReader] = []
        self._writer: Optional[StreamWriter] = None
        self._record_done = False

        self._vehicle: Optional[VehicleSimulator] = None
        self._imu: Optional[ImuSimulator] = None
        self._gnss: Optional[GnssSimulator] = None
        self._lidar: Optional[LidarSimulator] = None

        self._prepare_sources()

    # ----- catalog helpers -------------------------------------------------

    def _prepare_sources(self) -> None:
        mode = (self.cfg.mode or 'live').lower()
        if mode == 'replay':
            self._prepare_replay()
        else:
            self.cfg.mode = 'live'
            self._maps = resolve_playlist(self.cfg.map_ids)
            if not self.cfg.cycle_maps and self._maps:
                self._maps = [self._maps[0]]
            self._load_live_map(0)

    def _prepare_replay(self) -> None:
        readers: List[StreamReader] = []
        if self.cfg.stream_path:
            readers.append(StreamReader(self.cfg.stream_path))
        else:
            root = Path(self.cfg.streams_root) if self.cfg.streams_root else default_streams_root()
            catalog = {s['id']: s for s in discover_streams(root)}
            ids = list(self.cfg.stream_ids) if self.cfg.stream_ids else list(catalog.keys())
            if not ids:
                # Fall back to live if nothing recorded yet
                self.cfg.mode = 'live'
                self._maps = resolve_playlist(self.cfg.map_ids)
                self._load_live_map(0)
                return
            for sid in ids:
                if sid in catalog:
                    readers.append(StreamReader(catalog[sid]['path']))
                else:
                    # allow absolute/relative path entries
                    path = Path(sid)
                    if path.is_dir():
                        readers.append(StreamReader(path))
            if not readers:
                raise FileNotFoundError(f'no replay streams found under {root}')
        self._readers = readers
        if not self.cfg.cycle_maps:
            self._readers = [self._readers[0]]
        self._activate_reader(0)

    def _load_live_map(self, index: int) -> None:
        self._map_index = index % max(1, len(self._maps))
        spec = self._maps[self._map_index]
        self._active_map_id = spec.id
        self._active_stream_id = ''
        self._source_label = f'live:{spec.id}'
        speed = float(self.cfg.speed_mps if self.cfg.speed_mps is not None else spec.speed_mps)
        world = spec.build_world()
        self._vehicle = VehicleSimulator(speed_mps=speed, world=world, seed=spec.seed + 7)
        self._imu = ImuSimulator(seed=spec.seed + 11)
        self._gnss = GnssSimulator(seed=spec.seed + 13)
        self._lidar = LidarSimulator(world=world, seed=spec.seed + 17)
        self._vehicle.reset(0.0)

        if self.cfg.record_dir and not self._record_done and self._writer is None:
            out = Path(self.cfg.record_dir) / spec.id
            man = StreamManifest(
                stream_id=spec.id,
                map_id=spec.id,
                map_title=spec.title,
                duration_s=self.cfg.duration_s,
                speed_mps=speed,
                imu_hz=self.cfg.imu_hz,
                gnss_hz=self.cfg.gnss_hz,
                lidar_hz=self.cfg.lidar_hz,
                vehicle_hz=self.cfg.vehicle_hz,
                lidar_max_points=self.cfg.lidar_max_points,
                description=spec.description,
            )
            self._writer = StreamWriter(out, man)

    def _activate_reader(self, index: int) -> None:
        self._stream_index = index % max(1, len(self._readers))
        reader = self._readers[self._stream_index]
        self._active_map_id = reader.manifest.map_id or reader.manifest.stream_id
        self._active_stream_id = reader.manifest.stream_id or Path(reader.root).name
        self._source_label = f'replay:{self._active_stream_id}'
        # cursor indices into each series
        self._ri = {'odom': 0, 'imu': 0, 'gnss': 0, 'lidar': 0}
        self._reader = reader
        if reader.manifest.duration_s > 0:
            # keep configured duration but never shorter than stream
            self.cfg.duration_s = max(self.cfg.duration_s, float(reader.duration_s))

    # ----- pub/sub ---------------------------------------------------------

    def subscribe(self, callback: Subscriber) -> Callable[[], None]:
        """Register a tick listener. Returns unsubscribe()."""
        with self._lock:
            self._subscribers.append(callback)

        def _unsub() -> None:
            with self._lock:
                if callback in self._subscribers:
                    self._subscribers.remove(callback)

        return _unsub

    def _emit(self, tick: Dict[str, Any]) -> None:
        with self._lock:
            subs = list(self._subscribers)
        for cb in subs:
            try:
                cb(tick)
            except Exception:  # noqa: BLE001 — isolate bad subscribers
                pass

    # ----- lifecycle -------------------------------------------------------

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._running = True
        self._t = 0.0
        self._cycle = 0
        if self.cfg.mode == 'live' and self._vehicle is not None:
            self._vehicle.reset(0.0)
        self._thread = threading.Thread(target=self._loop, name='stream-hub', daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._writer is not None:
            self._writer.close()
            self._writer = None
            self._record_done = True

    def set_map(self, map_id: str) -> None:
        """Switch live map (clears history). Only valid in live mode."""
        if self.cfg.mode != 'live':
            raise RuntimeError('set_map only supported in live mode')
        spec = get_map(map_id)
        with self._lock:
            self._maps = [spec]
            self.cfg.cycle_maps = False
            self._clear_buffers_unlocked()
            self._load_live_map(0)
            self._t = 0.0
            self._cycle = 0
            self._segment_epoch += 1

    def set_stream(self, stream_id_or_path: str) -> None:
        """Switch replay stream (clears history)."""
        root = Path(self.cfg.streams_root) if self.cfg.streams_root else default_streams_root()
        catalog = {s['id']: s for s in discover_streams(root)}
        if stream_id_or_path in catalog:
            path = catalog[stream_id_or_path]['path']
        else:
            path = stream_id_or_path
        reader = StreamReader(path)
        with self._lock:
            self.cfg.mode = 'replay'
            self._readers = [reader]
            self.cfg.cycle_maps = False
            self._clear_buffers_unlocked()
            self._activate_reader(0)
            self._t = 0.0
            self._cycle = 0
            self._segment_epoch += 1

    def _clear_buffers_unlocked(self) -> None:
        self.imu_buf.clear()
        self.gnss_buf.clear()
        self.odom_buf.clear()
        self._last_lidar = None

    # ----- main loop -------------------------------------------------------

    def _loop(self) -> None:
        if self.cfg.mode == 'replay':
            self._loop_replay()
        else:
            self._loop_live()

    def _advance_cycle(self) -> None:
        self._cycle += 1
        with self._lock:
            self._clear_buffers_unlocked()
        if self.cfg.mode == 'live':
            if self._writer is not None:
                self._writer.close()
                self._writer = None
                self._record_done = True
            if self.cfg.cycle_maps and len(self._maps) > 1:
                self._load_live_map(self._map_index + 1)
            elif self._vehicle is not None:
                self._vehicle.reset(0.0)
        else:
            if self.cfg.cycle_maps and len(self._readers) > 1:
                self._activate_reader(self._stream_index + 1)
            else:
                self._activate_reader(self._stream_index)

    def _loop_live(self) -> None:
        cfg = self.cfg
        next_imu = 0.0
        next_gnss = 0.0
        next_lidar = 0.0
        next_odom = 0.0
        dt_odom = 1.0 / max(cfg.vehicle_hz, 1.0)
        t0_wall = time.perf_counter()
        seg_t0 = 0.0  # wall offset for current map segment
        epoch = self._segment_epoch

        while not self._stop.is_set():
            if self._segment_epoch != epoch:
                # Manual map switch — restart segment clock + sample schedule
                epoch = self._segment_epoch
                seg_t0 = time.perf_counter() - t0_wall
                next_imu = next_gnss = next_lidar = next_odom = 0.0

            wall = time.perf_counter() - t0_wall
            seg_wall = wall - seg_t0

            if cfg.loop and seg_wall >= cfg.duration_s:
                seg_t0 = wall
                next_imu = next_gnss = next_lidar = next_odom = 0.0
                self._advance_cycle()
                continue
            if not cfg.loop and seg_wall >= cfg.duration_s:
                self._running = False
                break

            self._t = float(seg_wall)
            assert self._vehicle and self._imu and self._gnss and self._lidar
            st = self._vehicle.step(self._t)

            if self._t + 1e-9 >= next_odom:
                next_odom = self._t + dt_odom
                odom = self._odom_dict(st)
                with self._lock:
                    self.odom_buf.append(odom)
                if self._writer:
                    self._writer.write('odom', odom)

            if self._t + 1e-9 >= next_imu:
                next_imu = self._t + 1.0 / max(cfg.imu_hz, 1.0)
                s = self._imu.sample(st)
                sample = {
                    't': s.t,
                    'gx': s.gx, 'gy': s.gy, 'gz': s.gz,
                    'ax': s.ax, 'ay': s.ay, 'az': s.az,
                }
                with self._lock:
                    self.imu_buf.append(sample)
                if self._writer:
                    self._writer.write('imu', sample)

            if self._t + 1e-9 >= next_gnss:
                next_gnss = self._t + 1.0 / max(cfg.gnss_hz, 1.0)
                g = self._gnss.sample(st)
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
                if self._writer:
                    self._writer.write('gnss', sample)

            if self._t + 1e-9 >= next_lidar:
                next_lidar = self._t + 1.0 / max(cfg.lidar_hz, 1.0)
                payload = self._lidar_payload(self._lidar.sample(self._vehicle, st))
                with self._lock:
                    self._last_lidar = payload
                if self._writer:
                    self._writer.write('lidar', payload)

            self._emit(self.live_tick())
            time.sleep(dt_odom)

    def _loop_replay(self) -> None:
        cfg = self.cfg
        dt = 1.0 / max(cfg.vehicle_hz, 1.0)
        t0_wall = time.perf_counter()
        seg_t0 = 0.0
        epoch = self._segment_epoch

        while not self._stop.is_set():
            if self._segment_epoch != epoch:
                epoch = self._segment_epoch
                seg_t0 = time.perf_counter() - t0_wall

            wall = time.perf_counter() - t0_wall
            seg_wall = wall - seg_t0
            duration = float(self._reader.duration_s)
            if duration <= 0.0:
                duration = float(cfg.duration_s)

            if cfg.loop and seg_wall >= duration:
                seg_t0 = wall
                self._advance_cycle()
                continue
            if not cfg.loop and seg_wall >= duration:
                self._running = False
                break

            self._t = float(seg_wall)
            self._feed_replay_upto(self._t)
            self._emit(self.live_tick())
            time.sleep(dt)

    def _feed_replay_upto(self, t: float) -> None:
        r = self._reader
        ri = self._ri

        def drain(kind: str, series: List[Dict[str, Any]], buf: Optional[Deque] = None, lidar: bool = False) -> None:
            i = ri[kind]
            while i < len(series) and float(series[i].get('t', 0.0)) <= t + 1e-9:
                sample = series[i]
                if lidar:
                    with self._lock:
                        self._last_lidar = sample
                elif buf is not None:
                    with self._lock:
                        buf.append(sample)
                i += 1
            ri[kind] = i

        drain('odom', r.odom, self.odom_buf)
        drain('imu', r.imu, self.imu_buf)
        drain('gnss', r.gnss, self.gnss_buf)
        drain('lidar', r.lidar, lidar=True)

    # ----- sample helpers --------------------------------------------------

    @staticmethod
    def _odom_dict(st: Any) -> Dict[str, Any]:
        speed = float(np.hypot(st.vx, st.vy))
        return {
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

    def _lidar_payload(self, fr: Any) -> Dict[str, Any]:
        pts = fr.points
        inten = fr.intensity
        max_n = self.cfg.lidar_max_points
        if pts.shape[0] > max_n:
            idx = np.linspace(0, pts.shape[0] - 1, max_n).astype(int)
            pts = pts[idx]
            inten = inten[idx]
        return {
            't': fr.t,
            'n': int(pts.shape[0]),
            'xy': np.round(pts[:, :2], 3).tolist(),
            'z': np.round(pts[:, 2], 3).tolist(),
            'i': np.round(inten.astype(float), 3).tolist(),
        }

    # ----- read APIs for subscribers --------------------------------------

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
            'mode': self.cfg.mode,
            'map_id': self._active_map_id,
            'stream_id': self._active_stream_id,
            'source': self._source_label,
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
            # Detectors only fill this in bus mode; keep shape stable for viz.
            'detect': {'clouds': {}, 'detections': None, 'alerts': [], 'vibe': None},
        }

    def live_tick(self) -> Dict[str, Any]:
        with self._lock:
            imu = self.imu_buf[-1] if self.imu_buf else None
            gnss = self.gnss_buf[-1] if self.gnss_buf else None
            odom = self.odom_buf[-1] if self.odom_buf else None
            lidar = self._last_lidar
            imu_tail = list(self.imu_buf)[-200:]
            odom_tail = list(self.odom_buf)[-300:]
            gnss_tail = list(self.gnss_buf)[-60:]
        return {
            'type': 'tick',
            't': self._t,
            'cycle': self._cycle,
            'running': self._running and not self._stop.is_set(),
            'mode': self.cfg.mode,
            'map_id': self._active_map_id,
            'stream_id': self._active_stream_id,
            'source': self._source_label,
            'imu': imu,
            'gnss': gnss,
            'odom': odom,
            'lidar': lidar,
            'imu_tail': imu_tail,
            'odom_tail': odom_tail,
            'gnss_tail': gnss_tail,
            'detect': {'clouds': {}, 'detections': None, 'alerts': [], 'vibe': None},
        }

    def catalog(self) -> Dict[str, Any]:
        root = Path(self.cfg.streams_root) if self.cfg.streams_root else default_streams_root()
        return {
            'mode': self.cfg.mode,
            'maps': list_maps(),
            'streams': discover_streams(root),
            'active': {
                'map_id': self._active_map_id,
                'stream_id': self._active_stream_id,
                'source': self._source_label,
                'cycle': self._cycle,
            },
            'playlist': {
                'maps': [m.id for m in self._maps],
                'streams': [
                    (r.manifest.stream_id or Path(r.root).name) for r in self._readers
                ],
                'cycle': self.cfg.cycle_maps,
            },
        }


def record_map_stream(
    map_id: str,
    out_dir: Path | str,
    *,
    duration_s: float = 60.0,
    imu_hz: float = 50.0,
    gnss_hz: float = 5.0,
    lidar_hz: float = 5.0,
    vehicle_hz: float = 50.0,
    lidar_max_points: int = 20000,
    speed_mps: Optional[float] = None,
) -> Path:
    """Synchronously synthesize one map into a replayable stream directory."""
    spec = get_map(map_id)
    speed = float(speed_mps if speed_mps is not None else spec.speed_mps)
    out = Path(out_dir)
    man = StreamManifest(
        stream_id=spec.id,
        map_id=spec.id,
        map_title=spec.title,
        duration_s=duration_s,
        speed_mps=speed,
        imu_hz=imu_hz,
        gnss_hz=gnss_hz,
        lidar_hz=lidar_hz,
        vehicle_hz=vehicle_hz,
        lidar_max_points=lidar_max_points,
        description=spec.description,
    )
    world = spec.build_world()
    vehicle = VehicleSimulator(speed_mps=speed, world=world, seed=spec.seed + 7)
    imu = ImuSimulator(seed=spec.seed + 11)
    gnss = GnssSimulator(seed=spec.seed + 13)
    lidar = LidarSimulator(world=world, seed=spec.seed + 17)
    vehicle.reset(0.0)

    next_imu = 0.0
    next_gnss = 0.0
    next_lidar = 0.0
    next_odom = 0.0
    dt = 1.0 / max(vehicle_hz, 1.0)

    with StreamWriter(out, man) as writer:
        t = 0.0
        while t <= duration_s + 1e-9:
            st = vehicle.step(t)
            if t + 1e-9 >= next_odom:
                next_odom = t + dt
                writer.write('odom', StreamHub._odom_dict(st))
            if t + 1e-9 >= next_imu:
                next_imu = t + 1.0 / max(imu_hz, 1.0)
                s = imu.sample(st)
                writer.write(
                    'imu',
                    {'t': s.t, 'gx': s.gx, 'gy': s.gy, 'gz': s.gz, 'ax': s.ax, 'ay': s.ay, 'az': s.az},
                )
            if t + 1e-9 >= next_gnss:
                next_gnss = t + 1.0 / max(gnss_hz, 1.0)
                g = gnss.sample(st)
                writer.write(
                    'gnss',
                    {
                        't': g.t,
                        'lat': g.latitude_deg,
                        'lon': g.longitude_deg,
                        'alt': g.altitude_m,
                        'fix_ok': g.fix_ok,
                        'status': g.status,
                        'x': st.x,
                        'y': st.y,
                    },
                )
            if t + 1e-9 >= next_lidar:
                next_lidar = t + 1.0 / max(lidar_hz, 1.0)
                fr = lidar.sample(vehicle, st)
                pts = fr.points
                inten = fr.intensity
                if pts.shape[0] > lidar_max_points:
                    idx = np.linspace(0, pts.shape[0] - 1, lidar_max_points).astype(int)
                    pts = pts[idx]
                    inten = inten[idx]
                writer.write(
                    'lidar',
                    {
                        't': fr.t,
                        'n': int(pts.shape[0]),
                        'xy': np.round(pts[:, :2], 3).tolist(),
                        'z': np.round(pts[:, 2], 3).tolist(),
                        'i': np.round(inten.astype(float), 3).tolist(),
                    },
                )
            t += dt
    return out
