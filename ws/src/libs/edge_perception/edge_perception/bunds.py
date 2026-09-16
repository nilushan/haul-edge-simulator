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
    height_m: np.ndarray


def _side_mask(points: np.ndarray, side: str, p: BundParams) -> np.ndarray:
    y = points[:, 1]
    # body: y left positive
    if side == 'left':
        center = +p.road_half_width_m
    else:
        center = -p.road_half_width_m
    return np.abs(y - center) <= p.shoulder_band_m


def extract_crest(points: np.ndarray, side: str, params: BundParams | None = None) -> BundCrest | None:
    p = params or BundParams()
    if points.size == 0:
        return None
    m = _side_mask(points, side, p)
    pts = points[m]
    if pts.shape[0] < p.min_points_per_bin:
        return None

    x = pts[:, 0]
    x0 = float(np.floor(x.min() / p.x_bin_m) * p.x_bin_m)
    bins = np.floor((x - x0) / p.x_bin_m).astype(np.int64)
    xs, ys, zs, hs = [], [], [], []
    for b in np.unique(bins):
        sel = bins == b
        if int(sel.sum()) < p.min_points_per_bin:
            continue
        chunk = pts[sel]
        # crest ≈ highest z in bin
        k = int(np.argmax(chunk[:, 2]))
        top = chunk[k]
        # height proxy vs local low
        h = float(top[2] - np.percentile(chunk[:, 2], 10.0))
        xs.append(float(top[0]))
        ys.append(float(top[1]))
        zs.append(float(top[2]))
        hs.append(h)
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
    """
    Returns (detections for viz, alert detail dicts for gaps/low sections).
    """
    p = params or BundParams()
    detections: List[Detection] = []
    alerts: List[dict] = []

    for side in ('left', 'right'):
        crest = extract_crest(points, side, p)
        if crest is None:
            alerts.append(
                {
                    'type': 'bund_gap',
                    'side': side,
                    'reason': 'no_crest',
                    'severity': 'warn',
                }
            )
            continue

        # one detection summarizing crest polyline
        detections.append(
            Detection(
                type='bund',
                confidence=0.8,
                frame_id='base_link',
                x=float(np.mean(crest.stations_x)),
                y=float(np.mean(crest.y)),
                z=float(np.mean(crest.z)),
                radius_m=0.0,
                extent_m=(
                    float(crest.stations_x.max() - crest.stations_x.min()),
                    float(p.shoulder_band_m),
                    float(np.mean(crest.height_m)),
                ),
                label=int(Label.BUND),
                point_count=int(crest.stations_x.size),
                details={
                    'side': side,
                    'mean_height_m': float(np.mean(crest.height_m)),
                    'min_height_m': float(np.min(crest.height_m)),
                    'stations': int(crest.stations_x.size),
                },
            )
        )

        # low segments
        low = crest.height_m < p.min_height_m
        if np.any(low):
            alerts.append(
                {
                    'type': 'bund_low',
                    'side': side,
                    'severity': 'warn',
                    'min_height_m': float(np.min(crest.height_m)),
                    'nom_height_m': p.nom_height_m,
                    'low_bins': int(np.count_nonzero(low)),
                }
            )

        # gap: large missing x span vs expected continuous bins
        if crest.stations_x.size >= 2:
            order = np.argsort(crest.stations_x)
            xs = crest.stations_x[order]
            gaps = np.diff(xs)
            big = gaps >= p.gap_length_m
            if np.any(big):
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
    """Return points that fall in left/right shoulder bands (for cloud publish)."""
    p = params or BundParams()
    if points.size == 0:
        return points
    m = _side_mask(points, 'left', p) | _side_mask(points, 'right', p)
    return points[m]
