"""Pure spatial tracking helpers for rock alert deduplication."""

from __future__ import annotations

import math
from collections import OrderedDict
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    z: float
    yaw: float


def body_to_map(x: float, y: float, z: float, pose: Pose2D) -> Tuple[float, float, float]:
    """Transform a body-frame point with a planar odometry pose."""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return (
        pose.x + c * x - s * y,
        pose.y + s * x + c * y,
        pose.z + z,
    )


class SpatialDeduplicator:
    """Bounded TTL cache of stationary-frame spatial cells."""

    def __init__(self, cell_m: float = 2.0, ttl_s: float = 120.0, capacity: int = 512) -> None:
        if cell_m <= 0 or ttl_s <= 0 or capacity < 1:
            raise ValueError('cell_m, ttl_s, and capacity must be positive')
        self.cell_m = float(cell_m)
        self.ttl_s = float(ttl_s)
        self.capacity = int(capacity)
        self._seen: OrderedDict[tuple[int, int], float] = OrderedDict()

    def _key(self, x: float, y: float) -> tuple[int, int]:
        return math.floor(x / self.cell_m), math.floor(y / self.cell_m)

    def seen_recently(self, x: float, y: float, now: float) -> bool:
        cutoff = now - self.ttl_s
        while self._seen and next(iter(self._seen.values())) < cutoff:
            self._seen.popitem(last=False)
        key = self._key(x, y)
        previous = self._seen.get(key)
        if previous is not None and previous >= cutoff:
            self._seen[key] = now
            self._seen.move_to_end(key)
            return True
        self._seen[key] = now
        self._seen.move_to_end(key)
        while len(self._seen) > self.capacity:
            self._seen.popitem(last=False)
        return False

    def __len__(self) -> int:
        return len(self._seen)
