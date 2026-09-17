"""Point-cloud helpers shared by LiDAR detectors (numpy only)."""

from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np


def _point_matrix(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    return points


def roi_mask(
    points: np.ndarray,
    *,
    x_min: float = 1.0,
    x_max: float = 60.0,
    # Wide enough to keep the far shoulder of a three-lane haul road in view.
    y_abs_max: float = 26.0,
    z_min: float = -3.0,
    z_max: float = 6.0,
) -> np.ndarray:
    """Boolean mask for a finite forward-corridor point in body frame."""
    points = _point_matrix(points)
    if points.shape[0] == 0:
        return np.zeros((0,), dtype=bool)
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    return (
        np.all(np.isfinite(points), axis=1)
        & (x >= x_min)
        & (x <= x_max)
        & (np.abs(y) <= y_abs_max)
        & (z >= z_min)
        & (z <= z_max)
    )


def voxel_downsample(points: np.ndarray, voxel_m: float = 0.15) -> np.ndarray:
    """Greedy one-point-per-voxel downsample of finite XYZ points."""
    points = _point_matrix(points)
    if points.shape[0] == 0:
        return points
    if not np.isfinite(voxel_m) or voxel_m <= 0:
        raise ValueError('voxel_m must be a positive finite value')
    points = points[np.all(np.isfinite(points), axis=1)]
    if points.shape[0] == 0:
        return points
    keys = np.floor(points / float(voxel_m)).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(idx)]


def euclidean_clusters(points: np.ndarray, eps_m: float, min_points: int) -> List[np.ndarray]:
    """
    Connected components of points within ``eps_m`` (true Euclidean distance).

    Neighbour lookup is restricted to the 27 grid cells of side ``eps_m`` around
    each seed, so this stays linear in point count instead of quadratic while
    returning the same clusters a brute-force scan would.
    """
    points = _point_matrix(points)
    if not np.isfinite(eps_m) or eps_m <= 0:
        raise ValueError('eps_m must be a positive finite value')
    if isinstance(min_points, bool) or int(min_points) != min_points or min_points < 1:
        raise ValueError('min_points must be a positive integer')
    min_points = int(min_points)
    n = points.shape[0]
    if n == 0:
        return []

    cells: Dict[Tuple[int, int, int], List[int]] = {}
    keys = np.floor(points / float(eps_m)).astype(np.int64)
    for i in range(n):
        cells.setdefault((int(keys[i, 0]), int(keys[i, 1]), int(keys[i, 2])), []).append(i)

    eps_sq = float(eps_m) * float(eps_m)
    unvisited = np.ones((n,), dtype=bool)
    clusters: List[np.ndarray] = []
    for seed in range(n):
        if not unvisited[seed]:
            continue
        unvisited[seed] = False
        members = [seed]
        queue = [seed]
        while queue:
            i = queue.pop()
            kx, ky, kz = int(keys[i, 0]), int(keys[i, 1]), int(keys[i, 2])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        bucket = cells.get((kx + dx, ky + dy, kz + dz))
                        if not bucket:
                            continue
                        for j in bucket:
                            if not unvisited[j]:
                                continue
                            delta = points[j] - points[i]
                            if float(delta @ delta) <= eps_sq:
                                unvisited[j] = False
                                members.append(j)
                                queue.append(j)
        if len(members) >= min_points:
            clusters.append(np.asarray(sorted(members), dtype=np.int64))
    return clusters


def pca_extents(points: np.ndarray) -> Tuple[float, float, float]:
    """Return full point spans along principal axes, largest first."""
    points = _point_matrix(points)
    points = points[np.all(np.isfinite(points), axis=1)]
    if points.shape[0] < 2:
        return (0.0, 0.0, 0.0)
    centered = points - points.mean(axis=0, keepdims=True)
    _, _, axes = np.linalg.svd(centered, full_matrices=False)
    projected = centered @ axes.T
    spans = np.ptp(projected, axis=0)
    spans = np.sort(spans)[::-1]
    if spans.size < 3:
        spans = np.pad(spans, (0, 3 - spans.size))
    return float(spans[0]), float(spans[1]), float(spans[2])
