"""Ground estimation and height-above-ground residuals."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class GroundResult:
    ground_idx: np.ndarray
    obstacle_idx: np.ndarray
    hag: np.ndarray  # height above ground per input point (nan if unknown)


def estimate_ground_grid(
    points: np.ndarray,
    *,
    cell_m: float = 0.75,
    ground_band_m: float = 0.18,
    min_cell_points: int = 2,
) -> GroundResult:
    """
    Simple elevation-grid ground model.

    For each XY cell keep the low percentile as ground, mark points near it
    as ground and anything higher as obstacle.
    """
    n = points.shape[0]
    if n == 0:
        empty = np.zeros((0,), dtype=np.int64)
        return GroundResult(empty, empty, np.zeros((0,), dtype=np.float32))

    xy = points[:, :2]
    z = points[:, 2]
    origin = xy.min(axis=0)
    keys = np.floor((xy - origin) / float(cell_m)).astype(np.int64)
    # pack 2D key
    kx = keys[:, 0] - keys[:, 0].min()
    ky = keys[:, 1] - keys[:, 1].min()
    flat = kx * (ky.max() + 1 + 1) + ky

    order = np.argsort(flat)
    flat_s = flat[order]
    z_s = z[order]

    ground_z = np.full(n, np.nan, dtype=np.float64)
    i = 0
    while i < n:
        j = i + 1
        while j < n and flat_s[j] == flat_s[i]:
            j += 1
        if (j - i) >= min_cell_points:
            gz = float(np.percentile(z_s[i:j], 20.0))
            ground_z[order[i:j]] = gz
        i = j

    hag = z - ground_z
    known = np.isfinite(hag)
    ground_mask = known & (np.abs(hag) <= ground_band_m)
    obstacle_mask = known & (hag > ground_band_m)

    return GroundResult(
        ground_idx=np.where(ground_mask)[0].astype(np.int64),
        obstacle_idx=np.where(obstacle_mask)[0].astype(np.int64),
        hag=hag.astype(np.float32),
    )
