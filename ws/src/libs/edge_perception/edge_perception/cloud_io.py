"""PointCloud2 encode/decode helpers used by detector nodes."""

from __future__ import annotations

import struct
from typing import Any, Optional, Tuple

import numpy as np

from edge_perception.schema import Label

_FLOAT32 = 7  # sensor_msgs.msg.PointField.FLOAT32 without importing ROS in the library


def decode_xyz(msg: Any) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Return ``(Nx3 float32 xyz, optional N intensity)`` from PointCloud2.

    XYZ and intensity fields must be scalar FLOAT32 values. Organized clouds,
    row padding, and either byte order are supported. Truncated trailing data is
    ignored rather than raising ``struct.error``.
    """
    field_list = list(msg.fields)
    field_names = [f.name for f in field_list]
    if len(field_names) != len(set(field_names)):
        raise ValueError('PointCloud2 field names must be unique')
    fields = {f.name: f for f in field_list}
    if not {'x', 'y', 'z'}.issubset(fields):
        return np.zeros((0, 3), dtype=np.float32), None

    required = [fields[name] for name in ('x', 'y', 'z')]
    if any(int(getattr(f, 'datatype', _FLOAT32)) != _FLOAT32 or int(getattr(f, 'count', 1)) != 1 for f in required):
        raise ValueError('x, y, and z PointCloud2 fields must be scalar FLOAT32 values')

    intensity_field = fields.get('intensity')
    if intensity_field is not None and (
        int(getattr(intensity_field, 'datatype', _FLOAT32)) != _FLOAT32
        or int(getattr(intensity_field, 'count', 1)) != 1
    ):
        raise ValueError('intensity PointCloud2 field must be a scalar FLOAT32 value')

    width = int(msg.width)
    height = int(msg.height)
    if width < 0 or height < 0:
        raise ValueError('PointCloud2 width and height cannot be negative')
    step = int(msg.point_step)
    if step <= 0:
        raise ValueError('PointCloud2 point_step must be positive')

    offsets = {name: int(field.offset) for name, field in fields.items()}
    ox, oy, oz = offsets['x'], offsets['y'], offsets['z']
    required_end = max(ox + 4, oy + 4, oz + 4)
    if min(ox, oy, oz) < 0 or required_end > step:
        raise ValueError('PointCloud2 XYZ field offsets exceed point_step')

    oi = offsets.get('intensity')
    if oi is not None and (oi < 0 or oi + 4 > step):
        raise ValueError('PointCloud2 intensity field offset exceeds point_step')

    row_step = int(getattr(msg, 'row_step', 0)) or width * step
    if row_step < width * step:
        raise ValueError('PointCloud2 row_step is smaller than width * point_step')

    data = memoryview(msg.data)
    n = width * height
    if n == 0:
        empty_points = np.zeros((0, 3), dtype=np.float32)
        empty_intensity = np.zeros((0,), dtype=np.float32) if oi is not None else None
        return empty_points, empty_intensity
    # Never allocate solely from untrusted dimensions. The byte buffer bounds
    # how many complete points could possibly be decoded.
    capacity = min(n, len(data) // step)
    pts = np.empty((capacity, 3), dtype=np.float32)
    inten = np.empty((capacity,), dtype=np.float32) if oi is not None else None
    fmt = '>f' if bool(getattr(msg, 'is_bigendian', False)) else '<f'

    valid = 0
    truncated = False
    for row in range(height):
        row_base = row * row_step
        for col in range(width):
            base = row_base + col * step
            if valid >= capacity or base + step > len(data):
                truncated = True
                break
            pts[valid, 0] = struct.unpack_from(fmt, data, base + ox)[0]
            pts[valid, 1] = struct.unpack_from(fmt, data, base + oy)[0]
            pts[valid, 2] = struct.unpack_from(fmt, data, base + oz)[0]
            if inten is not None:
                if base + oi + 4 > len(data):
                    truncated = True
                    break
                inten[valid] = struct.unpack_from(fmt, data, base + oi)[0]
            valid += 1
        if truncated:
            break

    return pts[:valid], inten[:valid] if inten is not None else None


def encode_xyz_label(
    points: np.ndarray,
    header: Any,
    *,
    intensity: np.ndarray | None = None,
    labels: np.ndarray | None = None,
    conf: np.ndarray | None = None,
) -> Any:
    """
    Build sensor_msgs/PointCloud2 with fields:
      x y z intensity label conf
    """
    from sensor_msgs.msg import PointCloud2, PointField  # lazy: nodes only

    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    n = int(points.shape[0])

    def _values(values: np.ndarray | None, default: float, name: str) -> np.ndarray:
        if values is None:
            return np.full((n,), default, dtype=np.float32)
        array = np.asarray(values, dtype=np.float32)
        if array.ndim != 1 or array.shape[0] != n:
            raise ValueError(f'{name} must have shape ({n},)')
        return array

    intensity = _values(intensity, 1.0, 'intensity')
    labels = _values(labels, float(Label.UNKNOWN), 'labels')
    conf = _values(conf, 1.0, 'conf')

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.is_bigendian = False
    msg.is_dense = bool(
        np.all(np.isfinite(points))
        and np.all(np.isfinite(intensity))
        and np.all(np.isfinite(labels))
        and np.all(np.isfinite(conf))
    )
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='label', offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name='conf', offset=20, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 24
    msg.row_step = 24 * n
    buf = bytearray(msg.row_step)
    for i in range(n):
        struct.pack_into(
            '<ffffff',
            buf,
            i * 24,
            float(points[i, 0]),
            float(points[i, 1]),
            float(points[i, 2]),
            float(intensity[i]),
            float(labels[i]),
            float(conf[i]),
        )
    msg.data = bytes(buf)
    return msg
