"""Ground estimation and height-above-ground residuals."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class GroundResult:
    ground_idx: np.ndarray
    obstacle_idx: np.ndarray
    hag: np.ndarray  # height above ground per input point (nan if unknown)


# One cell of lateral travel may legitimately drop by cell_m * max_slope; more
# than that is an object standing on the surface, not the surface itself.
_MAX_REFINE_CELLS = 4_000_000


def _pull_down_to_neighbours(
    ground_z: np.ndarray,
    cell_keys: np.ndarray,
    cell_z: np.ndarray,
    flat: np.ndarray,
    kx: np.ndarray,
    ky: np.ndarray,
    slope_allowance: float,
    passes: int = 2,
) -> np.ndarray:
    """Cap each cell estimate at its lowest neighbour plus one cell of slope."""
    width = int(ky.max()) + 1
    height = int(kx.max()) + 1
    if width <= 0 or height <= 0 or width * height > _MAX_REFINE_CELLS:
        return ground_z

    grid = np.full((height, width), np.inf, dtype=np.float64)
    grid[cell_keys // width, cell_keys % width] = cell_z
    for _ in range(max(int(passes), 1)):
        capped = grid.copy()
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)):
            shifted = np.full_like(grid, np.inf)
            src = grid[
                max(0, -dx): height - max(0, dx),
                max(0, -dy): width - max(0, dy),
            ]
            shifted[
                max(0, dx): height - max(0, -dx),
                max(0, dy): width - max(0, -dy),
            ] = src
            capped = np.minimum(capped, shifted + slope_allowance)
        grid = capped

    refined = grid[kx, ky]
    known = np.isfinite(ground_z)
    ground_z = ground_z.copy()
    ground_z[known] = np.minimum(ground_z[known], refined[known])
    return ground_z


def estimate_ground_grid(
    points: np.ndarray,
    *,
    cell_m: float = 0.75,
    ground_band_m: float = 0.18,
    min_cell_points: int = 2,
    max_slope: float = 0.35,
) -> GroundResult:
    """
    Estimate ground height per XY grid cell, then pull each cell down towards
    its neighbours.

    A cell-local estimate rides up anything that fills its own cell, which
    hides exactly the compact objects worth detecting — a rock covering one
    cell would read as ground. Each cell is therefore capped at the lowest
    neighbouring estimate plus what ``max_slope`` allows over one cell, so a
    rock is measured against the road beside it while real grade changes still
    pass through.

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
    cell_keys: list[int] = []
    cell_z: list[float] = []
    i = 0
    while i < finite_idx.size:
        j = i + 1
        while j < finite_idx.size and flat_s[j] == flat_s[i]:
            j += 1
        if (j - i) >= min_cell_points:
            gz = float(np.percentile(z_s[i:j], 20.0))
            ground_z[order[i:j]] = gz
            cell_keys.append(int(flat_s[i]))
            cell_z.append(gz)
        i = j

    if cell_keys and max_slope > 0:
        ground_z = _pull_down_to_neighbours(
            ground_z,
            np.asarray(cell_keys, dtype=np.int64),
            np.asarray(cell_z, dtype=np.float64),
            flat,
            kx,
            ky,
            float(cell_m) * float(max_slope),
        )

    hag[finite_idx] = z - ground_z
    known = np.isfinite(hag)
    ground_mask = known & (np.abs(hag) <= ground_band_m)
    obstacle_mask = known & (hag > ground_band_m)

    return GroundResult(
        ground_idx=np.flatnonzero(ground_mask).astype(np.int64),
        obstacle_idx=np.flatnonzero(obstacle_mask).astype(np.int64),
        hag=hag.astype(np.float32),
    )
