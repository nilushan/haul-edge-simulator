"""Shared haul-road world: centerline, berms/bunds, grades, rocks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np


@dataclass
class Rock:
    x: float
    y: float
    r: float


class HaulWorld:
    """
    ENU terrain for vehicle ground-truth and LiDAR hits.

    - Snaking haul corridor (centerline)
    - Bunds/berms along both shoulders
    - Climbs / drops along station
    - Rocks on/near the road
    """

    def __init__(
        self,
        road_half_width_m: float = 6.5,
        berm_height_m: float = 1.65,
        berm_width_m: float = 2.0,
        seed: int = 19,
    ) -> None:
        self.road_hw = float(road_half_width_m)
        self.berm_h = float(berm_height_m)
        self.berm_w = float(berm_width_m)
        self._rng = np.random.default_rng(seed)

        self.rocks: List[Rock] = []
        for i in range(48):
            s = 30.0 + i * 22.0 + float(self._rng.uniform(-6, 6))
            cx, cy, yaw = self.centerline(s)
            side = float(self._rng.choice([-1.0, 1.0]))
            if self._rng.random() < 0.45:
                lat = side * float(self._rng.uniform(0.8, self.road_hw - 0.6))
            else:
                lat = side * float(self._rng.uniform(self.road_hw - 0.3, self.road_hw + 1.5))
            nx, ny = -np.sin(yaw), np.cos(yaw)
            self.rocks.append(
                Rock(
                    x=float(cx + nx * lat),
                    y=float(cy + ny * lat),
                    r=float(self._rng.uniform(0.3, 1.15)),
                )
            )
        self._rock_xy = np.array([[r.x, r.y] for r in self.rocks], dtype=np.float64)
        self._rock_r = np.array([r.r for r in self.rocks], dtype=np.float64)

    def centerline(self, s: float | np.ndarray) -> tuple:
        """Centerline pose vs route station s [m] → (x, y, yaw)."""
        s_arr = np.asarray(s, dtype=float)
        y = (
            55.0 * np.sin(0.0065 * s_arr)
            + 22.0 * np.sin(0.0028 * s_arr + 0.7)
            + 8.0 * np.sin(0.014 * s_arr + 1.2)
        )
        x = s_arr + 12.0 * np.sin(0.004 * s_arr)
        ds = 0.75
        y2 = (
            55.0 * np.sin(0.0065 * (s_arr + ds))
            + 22.0 * np.sin(0.0028 * (s_arr + ds) + 0.7)
            + 8.0 * np.sin(0.014 * (s_arr + ds) + 1.2)
        )
        x2 = (s_arr + ds) + 12.0 * np.sin(0.004 * (s_arr + ds))
        yaw = np.arctan2(y2 - y, x2 - x)
        if np.isscalar(s):
            return float(x), float(y), float(yaw)
        return x, y, yaw

    def grade_z(self, s: float | np.ndarray) -> np.ndarray | float:
        s_arr = np.asarray(s, dtype=float)
        z = np.zeros_like(s_arr, dtype=float)
        z = z + 9.0 * self._smoothstep((s_arr - 60.0) / 140.0)  # climb
        z = z - 6.5 * self._smoothstep((s_arr - 260.0) / 70.0)  # drop
        z = z + 4.0 * self._smoothstep((s_arr - 380.0) / 90.0)  # climb out
        z = z - 2.5 * self._smoothstep((s_arr - 520.0) / 50.0)  # short drop
        z = z + 1.4 * np.sin(0.018 * s_arr)
        z = z + 0.7 * np.sin(0.04 * s_arr + 0.8)
        if np.isscalar(s):
            return float(z)
        return z

    @staticmethod
    def _smoothstep(t: np.ndarray | float) -> np.ndarray | float:
        t = np.clip(t, 0.0, 1.0)
        return t * t * (3.0 - 2.0 * t)

    def height(self, x: np.ndarray | float, y: np.ndarray | float) -> np.ndarray | float:
        """Vectorized surface height z(x, y)."""
        scalar = np.isscalar(x) and np.isscalar(y)
        x_arr = np.atleast_1d(np.asarray(x, dtype=float)).astype(float)
        y_arr = np.atleast_1d(np.asarray(y, dtype=float)).astype(float)

        # Station proxy: invert x ≈ s + 12 sin(0.004 s) with 1 fixed-point iter
        s = x_arr.copy()
        s = x_arr - 12.0 * np.sin(0.004 * s)
        cx, cy, yaw = self.centerline(s)
        nx = -np.sin(yaw)
        ny = np.cos(yaw)
        lat = (x_arr - cx) * nx + (y_arr - cy) * ny

        z = np.asarray(self.grade_z(s), dtype=float).copy()

        # road crown
        on_road = np.abs(lat) <= self.road_hw
        z = np.where(
            on_road,
            z + 0.05 * (1.0 - (lat / self.road_hw) ** 2),
            z - 0.12 * np.minimum(np.abs(lat) - self.road_hw, 2.5),
        )

        # bunds both sides
        for side in (-1.0, 1.0):
            c = side * (self.road_hw + 0.45 * self.berm_w)
            d = np.abs(lat - c)
            bund = self.grade_z(s) + self.berm_h * (1.0 - d / self.berm_w)
            z = np.where(d < self.berm_w, np.maximum(z, bund), z)

        # off-road roughness
        off = np.abs(lat) > self.road_hw + self.berm_w
        z = np.where(off, z + 0.35 * np.sin(0.03 * x_arr) * np.sin(0.05 * y_arr), z)

        # rocks
        if self._rock_xy.size:
            for i in range(self._rock_xy.shape[0]):
                rx, ry = self._rock_xy[i]
                rr = self._rock_r[i]
                d = np.hypot(x_arr - rx, y_arr - ry)
                disk = d < rr
                if np.any(disk):
                    top = z + np.sqrt(np.maximum(0.0, rr * rr - d * d))
                    # sit on grade under rock center
                    z = np.where(disk, np.maximum(z, top), z)

        if scalar:
            return float(z.reshape(-1)[0])
        return z.reshape(np.asarray(x).shape)

    def slope_along_heading(self, x: float, y: float, yaw: float, ds: float = 1.0) -> float:
        """dz/ds along heading (approx pitch of road)."""
        x2 = x + ds * np.cos(yaw)
        y2 = y + ds * np.sin(yaw)
        return float(self.height(x2, y2) - self.height(x, y)) / ds
