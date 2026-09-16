"""Bund / berm crest extraction along haul corridor shoulders."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from edge_perception.schema import Detection, Label


@dataclass
class BundParams:
    road_half_width_m: float = 6.5
    shoulder_band_m: float = 4.0
    x_bin_m: float = 1.5
    min_height_m: float = 0.45
    nom_height_m: float = 1.65
    gap_length_m: float = 10.0
    min_points_per_bin: int = 2


@dataclass
class BundCrest:
    side: str  # 'left' | 'right'
    stations_x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    height_m: np.ndarray  # NaN where local road-ground support is unavailable


def _side_mask(points: np.ndarray, side: str, p: BundParams) -> np.ndarray:
    if side not in ('left', 'right'):
        raise ValueError("side must be 'left' or 'right'")
    y = points[:, 1]
    # body: y left positive
    center = p.road_half_width_m if side == 'left' else -p.road_half_width_m
    return np.abs(y - center) <= p.shoulder_band_m


def extract_crest(points: np.ndarray, side: str, params: BundParams | None = None) -> BundCrest | None:
    """Extract shoulder crest points and heights relative to nearby road ground."""
    p = params or BundParams()
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    if p.x_bin_m <= 0 or p.shoulder_band_m < 0 or p.min_points_per_bin < 1:
        raise ValueError('x_bin_m and min_points_per_bin must be positive; shoulder_band_m cannot be negative')
    _side_mask(points, side, p)  # validate even for an empty cloud
    points = points[np.all(np.isfinite(points), axis=1)]
    if points.shape[0] == 0:
        return None

    shoulder = points[_side_mask(points, side, p)]
    if shoulder.shape[0] < p.min_points_per_bin:
        return None

    x0 = float(np.floor(shoulder[:, 0].min() / p.x_bin_m) * p.x_bin_m)
    bins = np.floor((shoulder[:, 0] - x0) / p.x_bin_m).astype(np.int64)
    xs, ys, zs, hs = [], [], [], []
    for b in np.unique(bins):
        selected = bins == b
        if int(selected.sum()) < p.min_points_per_bin:
            continue
        chunk = shoulder[selected]
        top = chunk[int(np.argmax(chunk[:, 2]))]

        bin_lo = x0 + float(b) * p.x_bin_m
        bin_hi = bin_lo + p.x_bin_m
        road = points[
            (points[:, 0] >= bin_lo)
            & (points[:, 0] < bin_hi)
            & (np.abs(points[:, 1]) <= p.road_half_width_m * 0.75)
        ]
        if road.shape[0] >= p.min_points_per_bin:
            ground_z = float(np.percentile(road[:, 2], 20.0))
            height = max(0.0, float(top[2]) - ground_z)
        else:
            height = float('nan')

        xs.append(float(top[0]))
        ys.append(float(top[1]))
        zs.append(float(top[2]))
        hs.append(height)

    if not xs:
        return None
    return BundCrest(
        side=side,
        stations_x=np.asarray(xs, dtype=np.float32),
        y=np.asarray(ys, dtype=np.float32),
        z=np.asarray(zs, dtype=np.float32),
        height_m=np.asarray(hs, dtype=np.float32),
    )


def detect_bunds(points: np.ndarray, params: BundParams | None = None) -> Tuple[List[Detection], List[dict]]:
    """Return crest detections and supported gap/low-section alert details."""
    p = params or BundParams()
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    finite_points = points[np.all(np.isfinite(points), axis=1)]
    detections: List[Detection] = []
    alerts: List[dict] = []

    for side in ('left', 'right'):
        side_points = finite_points[_side_mask(finite_points, side, p)]
        # Absence is evidence only when this specific shoulder was observed
        # over enough longitudinal distance; data from the opposite side does
        # not prove a physical gap here.
        side_coverage = (
            side_points.shape[0] >= 2 * p.min_points_per_bin
            and float(np.ptp(side_points[:, 0])) >= p.gap_length_m
        ) if side_points.shape[0] else False
        crest = extract_crest(finite_points, side, p)
        if crest is None:
            if side_coverage:
                alerts.append(
                    {
                        'type': 'bund_gap',
                        'side': side,
                        'reason': 'no_crest',
                        'severity': 'warn',
                    }
                )
            continue

        known_heights = crest.height_m[np.isfinite(crest.height_m)]
        height_extent = (
            float(np.max(known_heights))
            if known_heights.size
            else float(np.ptp(crest.z))
        )
        details = {
            'side': side,
            'mean_height_m': float(np.mean(known_heights)) if known_heights.size else None,
            'min_height_m': float(np.min(known_heights)) if known_heights.size else None,
            'height_bins': int(known_heights.size),
            'stations': int(crest.stations_x.size),
        }
        detections.append(
            Detection(
                type='bund',
                confidence=0.8 if known_heights.size else 0.55,
                frame_id='base_link',
                x=float(np.mean(crest.stations_x)),
                y=float(np.mean(crest.y)),
                z=float(np.mean(crest.z)),
                radius_m=0.0,
                extent_m=(
                    float(np.ptp(crest.stations_x)),
                    float(2.0 * p.shoulder_band_m),
                    height_extent,
                ),
                label=int(Label.BUND),
                point_count=int(crest.stations_x.size),
                details=details,
            )
        )

        # Only issue height alerts where a local road-ground reference exists.
        if known_heights.size:
            low = known_heights < p.min_height_m
            if np.any(low):
                alerts.append(
                    {
                        'type': 'bund_low',
                        'side': side,
                        'severity': 'warn',
                        'min_height_m': float(np.min(known_heights)),
                        'nom_height_m': p.nom_height_m,
                        'low_bins': int(np.count_nonzero(low)),
                    }
                )

        if crest.stations_x.size >= 2:
            xs = np.sort(crest.stations_x)
            gaps = np.diff(xs)
            if np.any(gaps >= p.gap_length_m):
                alerts.append(
                    {
                        'type': 'bund_gap',
                        'side': side,
                        'severity': 'warn',
                        'max_gap_m': float(np.max(gaps)),
                        'gap_length_m': p.gap_length_m,
                    }
                )

    return detections, alerts


def bund_points_from_detections(
    points: np.ndarray,
    params: BundParams | None = None,
) -> np.ndarray:
    """Return finite points in left/right shoulder bands for cloud publishing."""
    p = params or BundParams()
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    if points.shape[0] == 0:
        return points
    points = points[np.all(np.isfinite(points), axis=1)]
    mask = _side_mask(points, 'left', p) | _side_mask(points, 'right', p)
    return points[mask]
