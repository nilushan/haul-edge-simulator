"""IMU window features and excessive-vibration decisions."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np

from edge_perception.schema import AlertEvent, Severity


@dataclass
class ImuSample:
    t: float
    ax: float
    ay: float
    az: float


@dataclass
class VibeParams:
    window_s: float = 1.0
    hop_s: float = 0.2
    rms_warn: float = 1.5
    rms_crit: float = 2.5
    peak_warn: float = 4.0
    peak_crit: float = 6.0
    min_speed_mps: float = 1.0
    hold_s: float = 0.8
    min_samples: int = 10
    max_gap_s: float = 0.25


@dataclass
class VibeFeatures:
    t: float
    rms_az: float
    peak_az: float
    rms_horiz: float
    peak_horiz: float
    crest_az: float
    n: int


class VibrationMonitor:
    """
    Sliding-window IMU monitor.

    Call `push(sample)` for each IMU message; when enough time elapses a
    feature vector is produced. `evaluate` applies thresholds + debounce.
    """

    def __init__(self, params: VibeParams | None = None) -> None:
        self.params = params or VibeParams()
        p = self.params
        if p.window_s <= 0 or p.hop_s <= 0 or p.hold_s < 0:
            raise ValueError('window_s and hop_s must be positive; hold_s cannot be negative')
        if p.min_samples < 2 or p.max_gap_s <= 0:
            raise ValueError('min_samples must be at least 2 and max_gap_s must be positive')
        self._buf: Deque[ImuSample] = deque()
        self._last_emit_t = -1e9
        self._above_since: Optional[float] = None
        self._last_alert_t = -1e9
        self._last_sample_t: Optional[float] = None

    def reset(self) -> None:
        """Clear samples plus debounce/rate-limit state after a source reset."""
        self._buf.clear()
        self._last_emit_t = -1e9
        self._above_since = None
        self._last_alert_t = -1e9
        self._last_sample_t = None

    def push(self, sample: ImuSample) -> Optional[VibeFeatures]:
        values = (sample.t, sample.ax, sample.ay, sample.az)
        if not all(np.isfinite(value) for value in values):
            return None
        if self._last_sample_t is not None and (
            sample.t < self._last_sample_t
            or sample.t - self._last_sample_t > self.params.max_gap_s
        ):
            # Clock resets, replay loops, and forward data gaps invalidate both
            # the feature window and timestamp-based alert/debounce state.
            self.reset()
        self._last_sample_t = sample.t
        self._buf.append(sample)
        # drop old
        t_cut = sample.t - self.params.window_s * 1.5
        while self._buf and self._buf[0].t < t_cut:
            self._buf.popleft()

        if sample.t - self._last_emit_t < self.params.hop_s:
            return None
        window = [s for s in self._buf if s.t >= sample.t - self.params.window_s]
        if len(window) < self.params.min_samples:
            return None
        times = np.asarray([s.t for s in window], dtype=float)
        if times[-1] - times[0] < self.params.window_s * 0.8:
            return None
        if np.max(np.diff(times)) > self.params.max_gap_s:
            return None

        self._last_emit_t = sample.t
        return self._features(sample.t)

    def _features(self, t: float) -> VibeFeatures:
        t0 = t - self.params.window_s
        ax = np.array([s.ax for s in self._buf if s.t >= t0], dtype=float)
        ay = np.array([s.ay for s in self._buf if s.t >= t0], dtype=float)
        az = np.array([s.az for s in self._buf if s.t >= t0], dtype=float)
        # remove quasi-static gravity on z
        az_dyn = az - np.mean(az) if az.size else az
        horiz = np.hypot(ax - np.mean(ax) if ax.size else ax, ay - np.mean(ay) if ay.size else ay)
        rms_az = float(np.sqrt(np.mean(az_dyn ** 2))) if az_dyn.size else 0.0
        peak_az = float(np.max(np.abs(az_dyn))) if az_dyn.size else 0.0
        rms_h = float(np.sqrt(np.mean(horiz ** 2))) if horiz.size else 0.0
        peak_h = float(np.max(horiz)) if horiz.size else 0.0
        crest = peak_az / max(rms_az, 1e-6)
        return VibeFeatures(
            t=t,
            rms_az=rms_az,
            peak_az=peak_az,
            rms_horiz=rms_h,
            peak_horiz=peak_h,
            crest_az=float(crest),
            n=int(az.size),
        )

    def evaluate(
        self,
        feat: VibeFeatures,
        *,
        speed_mps: float = 0.0,
        vehicle_id: str = 'haul-01',
        source_node: str = 'edge_vibe_detect',
        pose: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> Optional[AlertEvent]:
        p = self.params
        feature_values = (
            feat.t, feat.rms_az, feat.peak_az, feat.rms_horiz,
            feat.peak_horiz, feat.crest_az, speed_mps, *pose,
        )
        if feat.n < p.min_samples or not all(np.isfinite(value) for value in feature_values):
            self._above_since = None
            return None
        if speed_mps < p.min_speed_mps:
            self._above_since = None
            return None

        sev = None
        if feat.rms_az >= p.rms_crit or feat.peak_az >= p.peak_crit:
            sev = Severity.CRITICAL
        elif feat.rms_az >= p.rms_warn or feat.peak_az >= p.peak_warn:
            sev = Severity.WARN

        if sev is None:
            self._above_since = None
            return None

        if self._above_since is None:
            self._above_since = feat.t
        held = feat.t - self._above_since
        if held < p.hold_s:
            return None
        # rate-limit alerts
        if feat.t - self._last_alert_t < max(p.hold_s, 1.0):
            return None
        self._last_alert_t = feat.t

        return AlertEvent(
            type='excessive_vibration',
            severity=sev,
            confidence=float(
                np.clip(
                    max(
                        feat.rms_az / max(p.rms_crit, 1e-3),
                        feat.peak_az / max(p.peak_crit, 1e-3),
                    ),
                    0.0,
                    1.0,
                )
            ),
            source_node=source_node,
            vehicle_id=vehicle_id,
            t_vehicle=feat.t,
            x=pose[0],
            y=pose[1],
            z=pose[2],
            geometry={'kind': 'window', 'window_s': p.window_s},
            details={
                'rms_az': feat.rms_az,
                'peak_az': feat.peak_az,
                'rms_horiz': feat.rms_horiz,
                'peak_horiz': feat.peak_horiz,
                'crest_factor': feat.crest_az,
                'duration_s': held,
                'speed_mps': speed_mps,
                'threshold': {
                    'rms_warn': p.rms_warn,
                    'rms_crit': p.rms_crit,
                    'peak_warn': p.peak_warn,
                    'peak_crit': p.peak_crit,
                },
                'n_samples': feat.n,
            },
        )

    def features_dict(self, feat: VibeFeatures) -> Dict:
        return {
            't': feat.t,
            'rms_az': feat.rms_az,
            'peak_az': feat.peak_az,
            'rms_horiz': feat.rms_horiz,
            'peak_horiz': feat.peak_horiz,
            'crest_az': feat.crest_az,
            'n': feat.n,
        }
