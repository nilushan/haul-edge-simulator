"""Pure point filtering helpers for the example processor."""

from __future__ import annotations

import numpy as np

from edge_perception.cloud_io import decode_xyz, encode_xyz_label


def process_points(points: np.ndarray, *, z_min: float, z_max: float, max_points: int) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    if not np.isfinite(z_min) or not np.isfinite(z_max) or z_min > z_max:
        raise ValueError('z_min and z_max must be finite with z_min <= z_max')
    if max_points < 1:
        raise ValueError('max_points must be positive')
    finite = np.all(np.isfinite(points), axis=1)
    selected = points[finite & (points[:, 2] >= z_min) & (points[:, 2] <= z_max)]
    if selected.shape[0] > max_points:
        indices = np.linspace(0, selected.shape[0] - 1, max_points).astype(int)
        selected = selected[indices]
    return selected


def encode_processed(points: np.ndarray, header: object) -> object:
    intensity = np.clip(0.5 + 0.2 * points[:, 2], 0.0, 1.0).astype(np.float32)
    return encode_xyz_label(points, header, intensity=intensity)


__all__ = ['decode_xyz', 'encode_processed', 'process_points']
