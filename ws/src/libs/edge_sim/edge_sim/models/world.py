"""Shared haul-road world: centerline, berms/bunds, grades, rocks."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

import numpy as np


@dataclass
class Rock:
    x: float
    y: float
    r: float


@dataclass(frozen=True)
class PathProfile:
    """Parametric corridor shape (used by map presets)."""

    y_terms: Tuple[Tuple[float, float, float], ...] = (
        (55.0, 0.0065, 0.0),
        (22.0, 0.0028, 0.7),
        (8.0, 0.014, 1.2),
    )
    x_wiggle_amp: float = 12.0
    x_wiggle_freq: float = 0.004
    grades: Tuple[Tuple[float, float, float], ...] = (
        (60.0, 9.0, 140.0),
        (260.0, -6.5, 70.0),
        (380.0, 4.0, 90.0),
        (520.0, -2.5, 50.0),
    )
    undulation: Tuple[Tuple[float, float, float], ...] = (
        (1.4, 0.018, 0.0),
        (0.7, 0.04, 0.8),
    )


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
        n_rocks: int = 48,
        rock_radius: Tuple[float, float] = (0.3, 1.15),
        path: PathProfile | None = None,
    ) -> None:
        self.road_hw = float(road_half_width_m)
        self.berm_h = float(berm_height_m)
        self.berm_w = float(berm_width_m)
        self.path = path or PathProfile()
        self._rng = np.random.default_rng(seed)

        self.rocks: List[Rock] = []
        r0, r1 = float(rock_radius[0]), float(rock_radius[1])
        for i in range(int(n_rocks)):
            s = 30.0 + i * 22.0 + float(self._rng.uniform(-6, 6))
            cx, cy, yaw = self.centerline(s)
            side = float(self._rng.choice([-1.0, 1.0]))
            if self._rng.random() < 0.45:
                lat = side * float(self._rng.uniform(0.8, max(0.9, self.road_hw - 0.6)))
            else:
                lat = side * float(self._rng.uniform(self.road_hw - 0.3, self.road_hw + 1.5))
            nx, ny = -np.sin(yaw), np.cos(yaw)
            self.rocks.append(
                Rock(
                    x=float(cx + nx * lat),
                    y=float(cy + ny * lat),
                    r=float(self._rng.uniform(r0, r1)),
                )
            )
        self._rock_xy = np.array([[r.x, r.y] for r in self.rocks], dtype=np.float64) if self.rocks else np.zeros((0, 2))
        self._rock_r = np.array([r.r for r in self.rocks], dtype=np.float64) if self.rocks else np.zeros((0,))

    def _xy_at(self, s_arr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        y = np.zeros_like(s_arr, dtype=float)
        for amp, freq, phase in self.path.y_terms:
            y = y + float(amp) * np.sin(float(freq) * s_arr + float(phase))
        x = s_arr + float(self.path.x_wiggle_amp) * np.sin(float(self.path.x_wiggle_freq) * s_arr)
        return x, y

    def centerline(self, s: float | np.ndarray) -> tuple:
        """Centerline pose vs route station s [m] → (x, y, yaw)."""
        s_arr = np.asarray(s, dtype=float)
        x, y = self._xy_at(s_arr)
        ds = 0.75
        x2, y2 = self._xy_at(s_arr + ds)
        yaw = np.arctan2(y2 - y, x2 - x)
        if np.isscalar(s):
            return float(x), float(y), float(yaw)
        return x, y, yaw

    def grade_z(self, s: float | np.ndarray) -> np.ndarray | float:
        s_arr = np.asarray(s, dtype=float)
        z = np.zeros_like(s_arr, dtype=float)
        for start, rise, length in self.path.grades:
            length = max(float(length), 1e-3)
            z = z + float(rise) * self._smoothstep((s_arr - float(start)) / length)
        for amp, freq, phase in self.path.undulation:
            z = z + float(amp) * np.sin(float(freq) * s_arr + float(phase))
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
        x_b, y_b = np.broadcast_arrays(np.asarray(x, dtype=float), np.asarray(y, dtype=float))
        shape = x_b.shape
        x_arr = np.atleast_1d(x_b).reshape(-1)
        y_arr = np.atleast_1d(y_b).reshape(-1)

        # Station proxy: invert x ≈ s + A sin(f s). A few fixed-point
        # iterations are cheap and remain accurate for more aggressive maps.
        amp = float(self.path.x_wiggle_amp)
        freq = float(self.path.x_wiggle_freq)
        s = x_arr.copy()
        for _ in range(5):
            s = x_arr - amp * np.sin(freq * s)
        cx, cy, yaw = self.centerline(s)
        nx = -np.sin(yaw)
        ny = np.cos(yaw)
        lat = (x_arr - cx) * nx + (y_arr - cy) * ny

        z_grade = np.asarray(self.grade_z(s), dtype=float)
        z = z_grade.copy()

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
            bund = z_grade + self.berm_h * (1.0 - d / self.berm_w)
            z = np.where(d < self.berm_w, np.maximum(z, bund), z)

        # off-road roughness
        off = np.abs(lat) > self.road_hw + self.berm_w
        z = np.where(off, z + 0.35 * np.sin(0.03 * x_arr) * np.sin(0.05 * y_arr), z)

        # Rocks are upper-hemisphere bumps on the local surface. Evaluate all
        # rocks in one bounded matrix instead of looping over the point cloud
        # once per rock; this keeps fine LiDAR marching practical.
        if self._rock_xy.size:
            dx = x_arr[:, None] - self._rock_xy[None, :, 0]
            dy = y_arr[:, None] - self._rock_xy[None, :, 1]
            radius_sq = self._rock_r[None, :] ** 2
            bump = np.sqrt(np.maximum(0.0, radius_sq - (dx * dx + dy * dy)))
            z = z + np.max(bump, axis=1)

        if scalar:
            return float(z.reshape(-1)[0])
        return z.reshape(shape)

    def slope_along_heading(self, x: float, y: float, yaw: float, ds: float = 1.0) -> float:
        """dz/ds along heading (approx pitch of road)."""
        if not np.isfinite(ds) or ds <= 0:
            raise ValueError('ds must be a positive finite value')
        x2 = x + ds * np.cos(yaw)
        y2 = y + ds * np.sin(yaw)
        return float(self.height(x2, y2) - self.height(x, y)) / ds
