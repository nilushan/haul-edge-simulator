"""PointCloud2 encode/decode helpers used by detector nodes."""

from __future__ import annotations

import struct
from typing import Any, Optional, Tuple

import numpy as np

from edge_perception.schema import Label


def decode_xyz(msg: Any) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Return (Nx3 float32 xyz, optional N intensity)."""
    offsets = {f.name: f.offset for f in msg.fields}
    if not {'x', 'y', 'z'}.issubset(offsets):
        return np.zeros((0, 3), dtype=np.float32), None
    step = int(msg.point_step)
    n = int(msg.width) * int(msg.height)
    data = memoryview(msg.data)
    ox, oy, oz = offsets['x'], offsets['y'], offsets['z']
    has_i = 'intensity' in offsets
    oi = offsets.get('intensity', 0)
    pts = np.zeros((n, 3), dtype=np.float32)
    inten = np.zeros((n,), dtype=np.float32) if has_i else None
    valid = 0
    for i in range(n):
        base = i * step
        if base + 12 > len(data):
            break
        x = struct.unpack_from('<f', data, base + ox)[0]
        y = struct.unpack_from('<f', data, base + oy)[0]
        z = struct.unpack_from('<f', data, base + oz)[0]
        pts[valid, 0] = x
        pts[valid, 1] = y
        pts[valid, 2] = z
        if inten is not None and base + oi + 4 <= len(data):
            inten[valid] = struct.unpack_from('<f', data, base + oi)[0]
        valid += 1
    if inten is not None:
        inten = inten[:valid]
    return pts[:valid], inten


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

    n = int(points.shape[0])
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.is_bigendian = False
    msg.is_dense = True
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
    if intensity is None:
        intensity = np.ones((n,), dtype=np.float32)
    if labels is None:
        labels = np.full((n,), float(Label.UNKNOWN), dtype=np.float32)
    if conf is None:
        conf = np.ones((n,), dtype=np.float32)
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
