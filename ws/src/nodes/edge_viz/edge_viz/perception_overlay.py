"""
Inline perception for hub mode: classify the hub's own scans for the browser.

In edge mode the detector nodes publish labelled clouds and alerts onto the ROS
bus and `TickBuffer` collects them. Hub mode has no bus, so the visualizer would
otherwise show raw geometry with no classes at all. This overlay wraps any tick
source and runs the same `edge_perception` pipeline the nodes run, so both paths
serve the identical taxonomy, colours, and event shapes.

It only *reads* the wrapped source — it never synthesizes sensor data.
"""

from __future__ import annotations

import logging
import math
from collections import deque
from typing import Any, Deque, Dict, List, Optional

import numpy as np

from edge_perception.schema import AlertEvent, DetectionSet
from edge_perception.semantics import SemanticParams, classify_frame
from edge_perception.tracking import SpatialDeduplicator
from edge_perception.vibration import ImuSample, VibeParams, VibrationMonitor

log = logging.getLogger('edge_viz.perception')

SOURCE_NODE = 'edge_viz_inline'


def body_to_map(x: float, y: float, z: float, odom: Dict[str, Any]) -> tuple[float, float, float]:
    """Body → ENU with the same R = Rz(yaw)·Ry(pitch)·Rx(roll) the sim uses."""
    yaw = float(odom.get('yaw') or 0.0)
    pitch = float(odom.get('pitch') or 0.0)
    roll = float(odom.get('roll') or 0.0)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    y1 = y * cr - z * sr
    z1 = y * sr + z * cr
    x2 = x * cp + z1 * sp
    z2 = -x * sp + z1 * cp
    return (
        float(odom.get('x') or 0.0) + x2 * cy - y1 * sy,
        float(odom.get('y') or 0.0) + x2 * sy + y1 * cy,
        float(odom.get('z') or 0.0) + z2,
    )


class PerceptionOverlay:
    """Tick source decorator that attaches classes, detections, and events."""

    def __init__(
        self,
        source: Any,
        *,
        params: Optional[SemanticParams] = None,
        vehicle_id: str = 'haul-01',
        alert_history: int = 40,
        dedup_cell_m: float = 3.0,
        bund_dedup_cell_m: float = 15.0,
        dedup_ttl_s: float = 90.0,
        max_points: int = 6000,
    ) -> None:
        if alert_history < 1 or max_points < 1:
            raise ValueError('alert_history and max_points must be positive')
        self._source = source
        self._params = params or SemanticParams()
        self._vehicle_id = str(vehicle_id)
        self._max_points = int(max_points)
        self._alerts: Deque[Dict[str, Any]] = deque(maxlen=int(alert_history))
        self._dedup_cell_m = float(dedup_cell_m)
        # A bund finding describes a stretch of shoulder, not a point: dedup it
        # over a segment-sized cell so one defect raises one event, not ten.
        self._bund_dedup_cell_m = float(bund_dedup_cell_m)
        self._dedup_ttl_s = float(dedup_ttl_s)
        # One cell map per event type: a rock and a bund issue can share a cell.
        self._dedup: Dict[str, SpatialDeduplicator] = {}
        self._vibe = VibrationMonitor(VibeParams())
        self._vibe_features: Optional[Dict[str, Any]] = None
        self._last_lidar_t: Optional[float] = None
        self._last_imu_t: Optional[float] = None
        self._labels: List[int] = []
        self._conf: List[float] = []
        self._detections: Optional[Dict[str, Any]] = None
        self._counts: Dict[str, int] = {}
        self._corridor: Dict[str, float] = {}
        self._sides: List[Dict[str, Any]] = []

    # --- delegation -------------------------------------------------------

    @property
    def cfg(self) -> Any:
        return self._source.cfg

    def start(self) -> None:
        self._source.start()

    def stop(self) -> None:
        self._source.stop()

    def catalog(self) -> Dict[str, Any]:
        return self._source.catalog()

    def set_map(self, map_id: str) -> None:
        self._reset()
        self._source.set_map(map_id)

    def set_stream(self, stream_id: str) -> None:
        self._reset()
        self._source.set_stream(stream_id)

    def snapshot(self) -> Dict[str, Any]:
        snap = self._source.snapshot()
        self._annotate(snap, latest_key='latest')
        return snap

    def live_tick(self) -> Dict[str, Any]:
        tick = self._source.live_tick()
        self._annotate(tick)
        return tick

    # --- perception -------------------------------------------------------

    def _reset(self) -> None:
        """Drop per-source state so a map switch cannot carry events across."""
        self._alerts.clear()
        self._dedup.clear()
        self._vibe.reset()
        self._vibe_features = None
        self._last_lidar_t = None
        self._last_imu_t = None
        self._labels = []
        self._conf = []
        self._detections = None
        self._counts = {}
        self._corridor = {}
        self._sides = []

    def _annotate(self, tick: Dict[str, Any], latest_key: Optional[str] = None) -> None:
        odom = tick.get('odom')
        if latest_key:
            odom = (tick.get(latest_key) or {}).get('odom')
        self._consume_imu(tick, odom)

        lidar = tick.get('lidar')
        if isinstance(lidar, dict) and lidar.get('xy'):
            t = lidar.get('t')
            if t != self._last_lidar_t:
                self._classify(lidar, odom)
                self._last_lidar_t = t
            if len(self._labels) == len(lidar.get('xy') or []):
                # Copy: the source caches this payload between ticks.
                lidar = {**lidar, 'label': self._labels, 'conf': self._conf}
                tick['lidar'] = lidar

        tick['detect'] = {
            'clouds': {},
            'detections': self._detections,
            'alerts': list(self._alerts),
            'vibe': self._vibe_features,
            'counts': dict(self._counts),
            'corridor': dict(self._corridor),
            'sides': list(self._sides),
            'source': SOURCE_NODE,
        }

    def _classify(self, lidar: Dict[str, Any], odom: Optional[Dict[str, Any]]) -> None:
        xy = lidar.get('xy') or []
        z = lidar.get('z') or []
        n = min(len(xy), len(z))
        if n == 0 or n > self._max_points:
            # Oversized frames would stall the tick pump; leave them unlabelled.
            if n > self._max_points:
                log.warning('skipping semantic pass for %d-point frame (max %d)', n, self._max_points)
            self._labels, self._conf, self._detections = [], [], None
            return

        points = np.empty((n, 3), dtype=np.float64)
        points[:, :2] = np.asarray(xy[:n], dtype=np.float64)
        points[:, 2] = np.asarray(z[:n], dtype=np.float64)
        t = float(lidar.get('t') or 0.0)
        try:
            result = classify_frame(
                points,
                params=self._params,
                t=t,
                vehicle_id=self._vehicle_id,
                source_node=SOURCE_NODE,
            )
        except ValueError as exc:
            log.warning('semantic pass failed: %s', exc)
            self._labels, self._conf, self._detections = [], [], None
            return

        self._labels = result.labels.astype(int).tolist()
        self._conf = np.round(result.conf.astype(float), 3).tolist()
        self._counts = result.counts
        self._corridor = result.corridor.to_dict()
        self._sides = [
            {'side': side.side, 'kind': side.kind, 'edge_m': side.edge_m, 'rise_m': side.rise_m}
            for side in result.sides
        ]
        self._detections = DetectionSet(
            t=t,
            frame_id='base_link',
            source_node=SOURCE_NODE,
            detections=result.detections,
        ).to_dict()

        for event in result.events:
            self._emit(event, odom, t)

    def _consume_imu(self, tick: Dict[str, Any], odom: Optional[Dict[str, Any]]) -> None:
        tail = tick.get('imu_tail')
        if not tail:
            history = tick.get('history')
            tail = (history or {}).get('imu') if isinstance(history, dict) else None
        if not tail:
            sample = tick.get('imu')
            tail = [sample] if isinstance(sample, dict) else []

        speed = float((odom or {}).get('speed') or 0.0)
        for sample in tail:
            if not isinstance(sample, dict):
                continue
            t = sample.get('t')
            if t is None:
                continue
            t = float(t)
            if self._last_imu_t is not None and t <= self._last_imu_t:
                continue
            self._last_imu_t = t
            features = self._vibe.push(
                ImuSample(
                    t=t,
                    ax=float(sample.get('ax') or 0.0),
                    ay=float(sample.get('ay') or 0.0),
                    az=float(sample.get('az') or 0.0),
                )
            )
            if features is None:
                continue
            self._vibe_features = {
                't': features.t,
                'rms_az': features.rms_az,
                'peak_az': features.peak_az,
                'rms_horiz': features.rms_horiz,
                'peak_horiz': features.peak_horiz,
                'crest_az': features.crest_az,
                'n': features.n,
            }
            event = self._vibe.evaluate(
                features,
                speed_mps=speed,
                vehicle_id=self._vehicle_id,
                source_node=SOURCE_NODE,
            )
            if event is not None:
                self._emit(event, odom, features.t, dedup=False)

    def _emit(
        self,
        event: AlertEvent,
        odom: Optional[Dict[str, Any]],
        t: float,
        *,
        dedup: bool = True,
    ) -> None:
        """Freeze a body-frame event into the world so its marker stops sliding."""
        if odom:
            event.x, event.y, event.z = body_to_map(event.x, event.y, event.z, odom)
            event.frame_id = 'map'
        if dedup:
            gate = self._dedup.get(event.type)
            if gate is None:
                cell_m = (
                    self._bund_dedup_cell_m
                    if event.type.startswith('bund')
                    else self._dedup_cell_m
                )
                gate = SpatialDeduplicator(cell_m=cell_m, ttl_s=self._dedup_ttl_s)
                self._dedup[event.type] = gate
            if gate.seen_recently(event.x, event.y, t):
                return
        event.t_vehicle = t
        self._alerts.appendleft(event.to_dict())
