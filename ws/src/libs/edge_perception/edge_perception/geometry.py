"""Point-cloud helpers shared by LiDAR detectors (numpy only)."""

from __future__ import annotations

from typing import Tuple

import numpy as np


def roi_mask(
    points: np.ndarray,
    *,
    x_min: float = 1.0,
    x_max: float = 60.0,
    y_abs_max: float = 12.0,
    z_min: float = -3.0,
    z_max: float = 6.0,
) -> np.ndarray:
    """Boolean mask for a forward haul-corridor ROI in body frame (ahead of vehicle)."""
    if points.size == 0:
        return np.zeros((0,), dtype=bool)
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    return (
        (x >= x_min)
        & (x <= x_max)
        & (np.abs(y) <= y_abs_max)
        & (z >= z_min)
        & (z <= z_max)
    )


def voxel_downsample(points: np.ndarray, voxel_m: float = 0.15) -> np.ndarray:
    """Greedy one-point-per-voxel downsample."""
    if points.size == 0 or voxel_m <= 0:
        return points
    keys = np.floor(points / float(voxel_m)).astype(np.int64)
    # unique rows
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(idx)]


def pca_extents(points: np.ndarray) -> Tuple[float, float, float]:
    """Return sorted eigenvalues of covariance (largest first) as extents proxy."""
    if points.shape[0] < 3:
        return (0.0, 0.0, 0.0)
    c = points - points.mean(axis=0, keepdims=True)
    cov = (c.T @ c) / max(points.shape[0] - 1, 1)
    w = np.linalg.eigvalsh(cov)
    w = np.sqrt(np.maximum(w, 0.0))
    w = np.sort(w)[::-1]
    return float(w[0]), float(w[1]), float(w[2])
