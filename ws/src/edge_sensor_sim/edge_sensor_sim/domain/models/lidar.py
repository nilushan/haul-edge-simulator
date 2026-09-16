"""Realistic multi-ring LiDAR via polar ray casting (not a rectangular grid)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .vehicle import VehicleSimulator, VehicleState
from .world import HaulWorld


@dataclass
class LidarFrame:
    t: float
    points: np.ndarray  # Nx3 float32 in body frame (m)
    intensity: np.ndarray  # N float32


class LidarSimulator:
    """
    Spinning multi-layer LiDAR emulation:

      - elevation rings + azimuth samples (polar pattern, not XY grid)
      - binary-search ray cast against the haul-world height field
      - range noise and random dropouts
      - beams origin at cab-roof mount

    Body frame: ROS x forward, y left, z up.
    """

    def __init__(
        self,
        n_rings: int = 16,
        n_azimuth: int = 180,  # 2° if full circle
        elev_min_deg: float = -18.0,
        elev_max_deg: float = 3.0,
        az_full_circle: bool = True,
        min_range_m: float = 1.5,
        max_range_m: float = 90.0,
        ray_iters: int = 12,
        noise_std_m: float = 0.03,
        dropout_prob: float = 0.025,
        world: HaulWorld | None = None,
        seed: int = 17,
        mount_xyz: tuple[float, float, float] = (2.5, 0.0, 3.2),
    ) -> None:
        self.world = world or HaulWorld(seed=seed + 5)
        self.min_range = float(min_range_m)
        self.max_range = float(max_range_m)
        self.ray_iters = int(ray_iters)
        self.noise_std = float(noise_std_m)
        self.dropout_prob = float(dropout_prob)
        self.mount = np.asarray(mount_xyz, dtype=float)
        self._rng = np.random.default_rng(seed)

        elevs = np.linspace(np.deg2rad(elev_min_deg), np.deg2rad(elev_max_deg), int(n_rings))
        if az_full_circle:
            azs = np.linspace(-np.pi, np.pi, int(n_azimuth), endpoint=False)
        else:
            azs = np.linspace(-np.deg2rad(110), np.deg2rad(110), int(n_azimuth))

        el_g, az_g = np.meshgrid(elevs, azs, indexing='ij')
        ce, se = np.cos(el_g), np.sin(el_g)
        ca, sa = np.cos(az_g), np.sin(az_g)
        # body: x fwd, y left, z up — az=0 forward
        dirs = np.stack([ce * ca, ce * sa, se], axis=-1).reshape(-1, 3)
        dirs /= np.linalg.norm(dirs, axis=1, keepdims=True).clip(min=1e-9)
        self._dir = dirs.astype(np.float64)
        self._n = dirs.shape[0]

    def sample(self, vehicle: VehicleSimulator, st: VehicleState) -> LidarFrame:
        R = vehicle.R_body_from_enu(st)
        origin = np.array([st.x, st.y, st.z], dtype=float)
        mount_enu = origin + R @ self.mount
        dir_enu = (R @ self._dir.T).T

        lo = np.full(self._n, self.min_range, dtype=float)
        hi = np.full(self._n, self.max_range, dtype=float)

        # Binary search hit distance for all rays (vectorized)
        for _ in range(self.ray_iters):
            mid = 0.5 * (lo + hi)
            pts = mount_enu[None, :] + dir_enu * mid[:, None]
            surf = np.asarray(self.world.height(pts[:, 0], pts[:, 1]), dtype=float)
            below = pts[:, 2] <= (surf + 0.06)
            hi = np.where(below, mid, hi)
            lo = np.where(~below, mid, lo)

        # Valid hit if final mid is under surface and not at max range only
        r_hit = 0.5 * (lo + hi)
        pts = mount_enu[None, :] + dir_enu * r_hit[:, None]
        surf = np.asarray(self.world.height(pts[:, 0], pts[:, 1]), dtype=float)
        hit = (pts[:, 2] <= surf + 0.12) & (r_hit < self.max_range * 0.995) & (r_hit > self.min_range)

        if not np.any(hit):
            return LidarFrame(
                t=st.t,
                points=np.zeros((0, 3), dtype=np.float32),
                intensity=np.zeros((0,), dtype=np.float32),
            )

        idx = np.where(hit)[0]
        # dropouts
        keep = self._rng.random(idx.size) >= self.dropout_prob
        idx = idx[keep]
        if idx.size == 0:
            return LidarFrame(
                t=st.t,
                points=np.zeros((0, 3), dtype=np.float32),
                intensity=np.zeros((0,), dtype=np.float32),
            )

        r = r_hit[idx] + self._rng.normal(0.0, self.noise_std, size=idx.size)
        r = np.clip(r, self.min_range, self.max_range)
        p_enu = mount_enu[None, :] + dir_enu[idx] * r[:, None]
        # snap to surface for ground consistency
        p_enu[:, 2] = np.asarray(self.world.height(p_enu[:, 0], p_enu[:, 1]), dtype=float)
        p_enu[:, 2] += self._rng.normal(0.0, self.noise_std * 0.4, size=idx.size)

        p_body = (R.T @ (p_enu - origin).T).T
        intensity = np.clip(1.15 - 0.01 * r, 0.12, 1.0).astype(np.float32)

        return LidarFrame(
            t=st.t,
            points=p_body.astype(np.float32),
            intensity=intensity,
        )
