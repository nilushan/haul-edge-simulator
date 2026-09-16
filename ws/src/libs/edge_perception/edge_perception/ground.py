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
    Estimate ground height independently in each XY grid cell.

    Non-finite points are left unclassified with a NaN height-above-ground
    value. Returned indices always refer to the original input array.
    """
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError('points must have shape (N, 3) or wider')
    if not np.isfinite(cell_m) or cell_m <= 0:
        raise ValueError('cell_m must be a positive finite value')
    if not np.isfinite(ground_band_m) or ground_band_m < 0:
        raise ValueError('ground_band_m must be a non-negative finite value')
    if isinstance(min_cell_points, bool) or int(min_cell_points) != min_cell_points or min_cell_points < 1:
        raise ValueError('min_cell_points must be a positive integer')
    min_cell_points = int(min_cell_points)

    n = points.shape[0]
    if n == 0:
        empty = np.zeros((0,), dtype=np.int64)
        return GroundResult(empty, empty, np.zeros((0,), dtype=np.float32))

    hag = np.full((n,), np.nan, dtype=np.float64)
    finite_idx = np.flatnonzero(np.all(np.isfinite(points[:, :3]), axis=1))
    if finite_idx.size == 0:
        empty = np.zeros((0,), dtype=np.int64)
        return GroundResult(empty, empty, hag.astype(np.float32))

    valid = points[finite_idx, :3]
    xy = valid[:, :2]
    z = valid[:, 2]
    origin = xy.min(axis=0)
    keys = np.floor((xy - origin) / float(cell_m)).astype(np.int64)
    # Pack a collision-free 2D key after shifting both axes to non-negative.
    kx = keys[:, 0] - keys[:, 0].min()
    ky = keys[:, 1] - keys[:, 1].min()
    flat = kx * (ky.max() + 1) + ky

    order = np.argsort(flat)
    flat_s = flat[order]
    z_s = z[order]

    ground_z = np.full(finite_idx.size, np.nan, dtype=np.float64)
    i = 0
    while i < finite_idx.size:
        j = i + 1
        while j < finite_idx.size and flat_s[j] == flat_s[i]:
            j += 1
        if (j - i) >= min_cell_points:
            gz = float(np.percentile(z_s[i:j], 20.0))
            ground_z[order[i:j]] = gz
        i = j

    hag[finite_idx] = z - ground_z
    known = np.isfinite(hag)
    ground_mask = known & (np.abs(hag) <= ground_band_m)
    obstacle_mask = known & (hag > ground_band_m)

    return GroundResult(
        ground_idx=np.flatnonzero(ground_mask).astype(np.int64),
        obstacle_idx=np.flatnonzero(obstacle_mask).astype(np.int64),
        hag=hag.astype(np.float32),
    )
