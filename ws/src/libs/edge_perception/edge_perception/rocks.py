"""Rock clustering / classification on non-ground points."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np

from edge_perception.geometry import euclidean_clusters, pca_extents
from edge_perception.schema import Detection, Label


@dataclass
class RockParams:
    cluster_eps_m: float = 0.7
    min_points: int = 8
    r_min_m: float = 0.25
    r_max_m: float = 1.6
    max_elongation: float = 3.2  # PCA major/minor
    hag_min_m: float = 0.2
    hag_max_m: float = 1.8
    min_range_m: float = 8.0  # body +X; drop near-field false positives


def detect_rocks(
    points: np.ndarray,
    hag: np.ndarray | None = None,
    *,
    params: RockParams | None = None,
) -> List[Detection]:
    """
    Cluster obstacle points and emit rock detections.

    `points` should already be non-ground (body frame). Optional `hag` is
    height-above-ground aligned with `points`.
    """
    p = params or RockParams()
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    if p.cluster_eps_m <= 0 or p.min_points < 1:
        raise ValueError('cluster_eps_m and min_points must be positive')
    if points.shape[0] == 0:
        return []

    if hag is not None:
        hag = np.asarray(hag)
        if hag.ndim != 1 or hag.shape[0] != points.shape[0]:
            raise ValueError('hag must have shape (N,) aligned with points')
    finite = np.all(np.isfinite(points), axis=1)
    if hag is not None:
        finite &= np.isfinite(hag)
        hag = hag[finite]
    points = points[finite]
    if points.shape[0] == 0:
        return []

    clusters = euclidean_clusters(points, p.cluster_eps_m, p.min_points)
    out: List[Detection] = []
    for idx in clusters:
        pts = points[idx]
        centroid = pts.mean(axis=0)
        radii = np.linalg.norm(pts - centroid, axis=1)
        r = float(np.percentile(radii, 90.0))
        e0, e1, e2 = pca_extents(pts)
        elong = (e0 / max(e1, 1e-3)) if e1 > 1e-6 else 99.0
        # Without an external ground model, use the cluster's lowest return as
        # a translation-invariant height proxy instead of absolute map/body Z.
        hag_vals = hag[idx] if hag is not None else pts[:, 2] - np.min(pts[:, 2])
        hag_mean = float(np.mean(hag_vals)) if np.size(hag_vals) else 0.0

        range_m = float(np.linalg.norm(centroid[:2]))
        if float(centroid[0]) < p.min_range_m or range_m < p.min_range_m:
            continue
        if not (p.r_min_m <= r <= p.r_max_m):
            continue
        if elong > p.max_elongation:
            continue
        if not (p.hag_min_m <= hag_mean <= p.hag_max_m):
            continue

        # crude confidence from size + compactness
        conf = float(
            np.clip(
                0.55
                + 0.25 * (1.0 - abs(r - 0.6) / 0.6)
                + 0.2 * (1.0 - min(elong, p.max_elongation) / p.max_elongation),
                0.0,
                1.0,
            )
        )
        out.append(
            Detection(
                type='rock',
                confidence=conf,
                frame_id='base_link',
                x=float(centroid[0]),
                y=float(centroid[1]),
                z=float(centroid[2]),
                radius_m=r,
                extent_m=(e0, e1, e2),
                label=int(Label.ROCK),
                point_count=int(idx.size),
                details={
                    'hag_mean_m': hag_mean,
                    'elongation': float(elong),
                    'range_m': float(np.linalg.norm(centroid[:2])),
                    'lateral_m': float(centroid[1]),
                },
            )
        )
    return out
