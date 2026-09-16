"""Pure conversion helpers for the ROS sensor-source bridge."""

from __future__ import annotations

from typing import Any, Dict

import numpy as np


def lidar_arrays(lidar: Dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    """Validate a StreamHub lidar payload and return aligned XYZ/intensity arrays."""
    xy = np.asarray(lidar.get('xy') or [], dtype=np.float32)
    if xy.size == 0:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0,), dtype=np.float32)
    if xy.ndim == 1:
        if xy.size % 2:
            raise ValueError('lidar xy must contain pairs')
        xy = xy.reshape(-1, 2)
    if xy.ndim != 2 or xy.shape[1] != 2:
        raise ValueError('lidar xy must have shape (N, 2)')

    n = int(xy.shape[0])
    points = np.zeros((n, 3), dtype=np.float32)
    points[:, :2] = xy

    z = np.asarray(lidar.get('z') or [], dtype=np.float32).reshape(-1)
    if z.size:
        count = min(n, int(z.size))
        points[:count, 2] = z[:count]

    intensity = np.ones((n,), dtype=np.float32)
    supplied = np.asarray(lidar.get('i') or [], dtype=np.float32).reshape(-1)
    if supplied.size:
        count = min(n, int(supplied.size))
        intensity[:count] = supplied[:count]
    return points, intensity


def xyzi_bytes(points: np.ndarray, intensity: np.ndarray) -> bytes:
    """Pack aligned XYZ and intensity arrays as little-endian float32 rows."""
    points = np.asarray(points, dtype=np.float32)
    intensity = np.asarray(intensity, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    if intensity.ndim != 1 or intensity.shape[0] != points.shape[0]:
        raise ValueError('intensity must have shape (N,) aligned with points')
    packed = np.empty((points.shape[0], 4), dtype='<f4')
    packed[:, :3] = points
    packed[:, 3] = intensity
    return packed.tobytes(order='C')
