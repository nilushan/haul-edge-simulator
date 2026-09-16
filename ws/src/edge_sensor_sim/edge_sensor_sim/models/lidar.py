"""Synthetic LiDAR: ground plane, berms, rocks in body/lidar frame."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from .vehicle import VehicleSimulator, VehicleState


@dataclass
class LidarFrame:
    t: float
    points: np.ndarray  # Nx3 float32, lidar/body frame (m)
    intensity: np.ndarray  # N float32


class LidarSimulator:
    """
    Generates a forward haul-road cloud each tick:
      - ground surface with mild undulation
      - left/right berms
      - fixed world rocks (spheres on the ground)

    Implemented as a structured sample of the height field (fast enough for ~10 Hz),
    not a full GPU ray tracer.
    """

    def __init__(
        self,
        n_forward: int = 80,
        n_lateral: int = 60,
        forward_min_m: float = 2.0,
        forward_max_m: float = 50.0,
        lateral_span_m: float = 12.0,
        noise_std_m: float = 0.02,
        n_rocks: int = 10,
        berm_height_m: float = 1.4,
        berm_offset_y_m: float = 7.5,
        seed: int = 17,
    ) -> None:
        self.n_fwd = int(n_forward)
        self.n_lat = int(n_lateral)
        self.fwd_min = float(forward_min_m)
        self.fwd_max = float(forward_max_m)
        self.lat_span = float(lateral_span_m)
        self.noise_std = float(noise_std_m)
        self.berm_h = float(berm_height_m)
        self.berm_y = float(berm_offset_y_m)
        self._rng = np.random.default_rng(seed)

        self._rocks: List[Tuple[float, float, float]] = []
        for i in range(int(n_rocks)):
            sx = 20.0 + 10.0 * i + float(self._rng.uniform(-2.5, 2.5))
            sy = float(self._rng.uniform(-5.5, 5.5))
            sr = float(self._rng.uniform(0.3, 0.85))
            self._rocks.append((sx, sy, sr))

    def _ground_z(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        return 0.05 * np.sin(0.08 * x) + 0.02 * np.sin(0.11 * y)

    def _height_field(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        z = self._ground_z(x, y)
        # Berms
        for sign in (1.0, -1.0):
            cy = sign * self.berm_y
            d = np.abs(y - cy)
            mask = d < 0.9
            z = np.where(mask, z + self.berm_h * (1.0 - d / 0.9), z)
        # Rocks
        for rx, ry, rr in self._rocks:
            d = np.hypot(x - rx, y - ry)
            disk = d < rr
            if np.any(disk):
                rock_z = self._ground_z(np.array([rx]), np.array([ry]))[0] + np.sqrt(
                    np.maximum(0.0, rr * rr - d * d)
                )
                z = np.where(disk, np.maximum(z, rock_z), z)
        return z

    def sample(self, vehicle: VehicleSimulator, st: VehicleState) -> LidarFrame:
        # Sample a grid in body XY (forward x, left y), lift to world height, back to body
        xs = np.linspace(self.fwd_min, self.fwd_max, self.n_fwd)
        ys = np.linspace(-self.lat_span, self.lat_span, self.n_lat)
        xx, yy = np.meshgrid(xs, ys, indexing='xy')
        # body points on z=0 plane as seeds
        body = np.stack([xx.ravel(), yy.ravel(), np.zeros(xx.size)], axis=1)

        # to ENU
        R = vehicle.R_body_from_enu(st)
        origin = np.array([st.x, st.y, st.z], dtype=float)
        enu = (R @ body.T).T + origin
        enu[:, 2] = self._height_field(enu[:, 0], enu[:, 1])

        # back to body
        body_pts = (R.T @ (enu - origin).T).T
        if self.noise_std > 0:
            body_pts = body_pts + self._rng.normal(0.0, self.noise_std, size=body_pts.shape)

        # drop points behind or too close/far
        rng = np.linalg.norm(body_pts, axis=1)
        keep = (body_pts[:, 0] > 0.5) & (rng > 1.0) & (rng < self.fwd_max + 5.0)
        body_pts = body_pts[keep]

        intensity = (0.2 + 0.6 * np.clip(np.abs(body_pts[:, 2]) / 2.0, 0.0, 1.0)).astype(
            np.float32
        )
        return LidarFrame(
            t=st.t,
            points=body_pts.astype(np.float32),
            intensity=intensity,
        )
