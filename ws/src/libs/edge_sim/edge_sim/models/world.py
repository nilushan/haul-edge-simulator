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


@dataclass
class BundDefect:
    """A stretch of shoulder where the berm is under-built or absent."""

    side: float  # -1.0 right of centreline, +1.0 left
    s_start: float  # route station [m]
    length_m: float
    height_scale: float  # 0.0 = no berm at all
    ramp_m: float = 3.0

    @property
    def kind(self) -> str:
        return 'gap' if self.height_scale <= 0.05 else 'low'


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
    # Road half-width variation along the route: (amplitude_m, freq_1/m, phase)
    width_terms: Tuple[Tuple[float, float, float], ...] = (
        (2.2, 0.0045, 0.0),
        (1.1, 0.011, 1.3),
    )
    # How often the road crosses from one side of the hill to the other, and
    # how sharp that transition is.
    tilt_period_m: float = 460.0
    tilt_phase: float = 0.35
    tilt_sharpness: float = 2.6


class HaulWorld:
    """
    ENU terrain for vehicle ground-truth and LiDAR hits.

    - Snaking haul corridor (centerline)
    - Bunds/berms along both shoulders
    - Climbs / drops along station
    - Rocks on/near the road
    - Optional bund defects (low or missing berm sections)
    """

    def __init__(
        self,
        road_half_width_m: float = 13.0,
        berm_height_m: float = 1.65,
        berm_width_m: float = 2.6,
        seed: int = 19,
        n_rocks: int = 48,
        rock_radius: Tuple[float, float] = (0.3, 1.15),
        path: PathProfile | None = None,
        n_bund_defects: int = 4,
        cut_slope: float = 0.75,
        cut_height_m: float = 9.0,
        fill_slope: float = 0.62,
        fill_depth_m: float = 14.0,
        lane_offset_frac: float = 0.45,
    ) -> None:
        self.road_hw = float(road_half_width_m)
        self.berm_h = float(berm_height_m)
        self.berm_w = float(berm_width_m)
        # A haul road is cut into a hillside: one shoulder runs up into the
        # batter, the other drops away and carries the safety bund.
        self.cut_slope = float(cut_slope)
        self.cut_height = float(cut_height_m)
        self.fill_slope = float(fill_slope)
        self.fill_depth = float(fill_depth_m)
        # Trucks keep to one side; the far bund is much further away than the
        # near one, which is what the detectors actually have to cope with.
        self.lane_offset_frac = float(lane_offset_frac)
        self.path = path or PathProfile()
        self._rng = np.random.default_rng(seed)

        self.rocks: List[Rock] = []
        r0, r1 = float(rock_radius[0]), float(rock_radius[1])
        for i in range(int(n_rocks)):
            s = 30.0 + i * 22.0 + float(self._rng.uniform(-6, 6))
            cx, cy, yaw = self.centerline(s)
            half_width = float(self.road_half_width(s))
            lane = float(self.lane_offset(s))
            roll = float(self._rng.random())
            if roll < 0.5:
                # In the travelled lane, where a truck would actually meet it.
                lat = lane + float(self._rng.uniform(-3.0, 3.0))
            elif roll < 0.7:
                # Anywhere across the running surface.
                lat = float(self._rng.uniform(-half_width + 0.8, half_width - 0.8))
            else:
                # Spalled off a shoulder or the batter toe.
                side = float(self._rng.choice([-1.0, 1.0]))
                lat = side * float(self._rng.uniform(half_width - 1.5, half_width + 1.0))
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

        # Bund defects give the edge detectors something real to find: without
        # them every berm in the world is compliant by construction.
        self.bund_defects: List[BundDefect] = []
        for i in range(max(int(n_bund_defects), 0)):
            # A defect only means anything on a shoulder that carries a bund,
            # so each one is moved to the nearest station where its side is the
            # falling side. Most go on the side the truck drives nearest; one
            # in four sits on the far shoulder, which a single pass cannot
            # measure — that case is real and the detectors must not pretend
            # otherwise.
            near_side = 1.0 if self.lane_offset_frac >= 0 else -1.0
            side = near_side if i % 4 != 3 else -near_side
            gap = i % 3 == 2
            target = 90.0 + i * 130.0 + float(self._rng.uniform(-25.0, 25.0))
            self.bund_defects.append(
                BundDefect(
                    side=side,
                    s_start=self._nearest_fill_station(target, side),
                    length_m=float(self._rng.uniform(14.0, 34.0)),
                    height_scale=0.0 if gap else float(self._rng.uniform(0.22, 0.52)),
                )
            )

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

    def _nearest_fill_station(
        self,
        target_s: float,
        side: float,
        span_m: float = 500.0,
        clearance_m: float = 60.0,
    ) -> float:
        """
        Closest station to ``target_s`` where ``side`` is the falling side.

        Stations already carrying a defect are skipped so findings stay
        separable instead of piling up in one window.
        """
        taken = [(d.s_start, d.length_m) for d in self.bund_defects]
        for offset in np.arange(0.0, float(span_m), 5.0):
            for candidate in (target_s + offset, target_s - offset):
                if candidate < 20.0:
                    continue
                tilt = float(self.hill_tilt(candidate))
                # tilt > 0 means the hill rises to the left, so the left
                # shoulder is a cut and the right one is fill.
                is_fill = (side > 0 and tilt <= -0.35) or (side < 0 and tilt >= 0.35)
                if not is_fill:
                    continue
                if any(
                    candidate < start + length + clearance_m
                    and start < candidate + clearance_m
                    for start, length in taken
                ):
                    continue
                return float(candidate)
        return float(target_s)

    def road_half_width(self, s: float | np.ndarray) -> np.ndarray | float:
        """Half-width of the running surface at station ``s`` [m]."""
        s_arr = np.asarray(s, dtype=float)
        hw = np.full_like(s_arr, self.road_hw, dtype=float)
        for amp, freq, phase in self.path.width_terms:
            hw = hw + float(amp) * np.sin(float(freq) * s_arr + float(phase))
        hw = np.maximum(hw, 4.0)
        if np.isscalar(s):
            return float(hw.reshape(-1)[0]) if hw.ndim else float(hw)
        return hw

    def lane_offset(self, s: float | np.ndarray) -> np.ndarray | float:
        """Lateral offset of the travelled lane centre (positive = left)."""
        hw = np.asarray(self.road_half_width(s), dtype=float)
        offset = hw * self.lane_offset_frac
        if np.isscalar(s):
            return float(np.asarray(offset).reshape(-1)[0])
        return offset

    def hill_tilt(self, s: float | np.ndarray) -> np.ndarray | float:
        """
        Cross-slope of the hillside at ``s``: +1 rises to the left, -1 to the
        right, ~0 on a flat bench where both shoulders are fill.
        """
        s_arr = np.asarray(s, dtype=float)
        wave = np.sin(2.0 * np.pi * s_arr / max(self.path.tilt_period_m, 1.0) + self.path.tilt_phase)
        tilt = np.tanh(self.path.tilt_sharpness * wave)
        if np.isscalar(s):
            return float(np.asarray(tilt).reshape(-1)[0])
        return tilt

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

    def _berm_scale(self, s_arr: np.ndarray, side: float) -> np.ndarray:
        """Per-station berm height multiplier for one side (1.0 = as designed)."""
        scale = np.ones_like(np.asarray(s_arr, dtype=float))
        for defect in self.bund_defects:
            if defect.side != side:
                continue
            ramp = max(float(defect.ramp_m), 1e-3)
            rise = self._smoothstep((s_arr - defect.s_start) / ramp)
            fall = self._smoothstep((defect.s_start + defect.length_m - s_arr) / ramp)
            inside = np.clip(np.minimum(rise, fall), 0.0, 1.0)
            scale = np.minimum(scale, 1.0 - inside * (1.0 - defect.height_scale))
        return scale

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
        half_width = np.asarray(self.road_half_width(s), dtype=float)
        tilt = np.asarray(self.hill_tilt(s), dtype=float)

        # Running surface: crowned for drainage, level across its width.
        on_road = np.abs(lat) <= half_width
        z = np.where(
            on_road,
            z_grade + 0.05 * (1.0 - (lat / half_width) ** 2),
            z_grade,
        )

        # Beyond the edge the hillside either climbs (cut batter) or falls away
        # (fill batter). `up` blends between the two so the road can cross from
        # one side of the hill to the other without a step in the terrain.
        beyond = np.maximum(np.abs(lat) - half_width, 0.0)
        left = lat >= 0.0
        up = np.where(left, np.clip(tilt, 0.0, 1.0), np.clip(-tilt, 0.0, 1.0))
        rise = np.minimum(self.cut_slope * beyond, self.cut_height)
        drop = np.minimum(self.fill_slope * beyond, self.fill_depth)
        z = np.where(on_road, z, z_grade + up * rise - (1.0 - up) * drop)

        # Safety bund on the falling shoulder only; a cut batter needs none, so
        # the berm fades out as that side becomes the high side.
        c = half_width + 0.45 * self.berm_w
        d = np.abs(np.abs(lat) - c)
        defect_scale = np.where(left, self._berm_scale(s, 1.0), self._berm_scale(s, -1.0))
        # Present on a falling shoulder, gone once the side is clearly a batter.
        # The handover is deliberately short: a real bund is built to height or
        # not built at all, it is not tapered away over a hundred metres.
        presence = self._smoothstep((0.40 - up) / 0.10)
        scale = defect_scale * presence
        bund = z_grade + self.berm_h * scale * (1.0 - d / self.berm_w)
        z = np.where((d < self.berm_w) & (scale > 0.02), np.maximum(z, bund), z)

        # off-road roughness
        off = np.abs(lat) > half_width + self.berm_w
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
