"""Haul-truck motion along the world centerline (turns, climbs, drops)."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .world import HaulWorld


@dataclass
class VehicleState:
    """Pose and motion in a local ENU frame (metres, radians, SI)."""

    t: float
    x: float
    y: float
    z: float
    yaw: float
    pitch: float
    roll: float
    vx: float
    vy: float
    vz: float
    yaw_rate: float
    pitch_rate: float
    roll_rate: float
    ax: float
    ay: float
    az: float


def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _exp_smooth(prev: float, target: float, dt: float, tau: float) -> float:
    if tau <= 1e-6:
        return target
    a = 1.0 - math.exp(-dt / tau)
    return prev + a * (target - prev)


class VehicleSimulator:
    """
    Follows HaulWorld centerline with:
      - speed changes on grades
      - heading from route curvature (real turns)
      - pitch from road slope (climbs/drops)
      - roll from lateral accel in turns
      - occasional surface bumps
    """

    def __init__(
        self,
        speed_mps: float = 7.5,
        world: HaulWorld | None = None,
        seed: int = 7,
    ) -> None:
        if not math.isfinite(speed_mps) or speed_mps <= 0:
            raise ValueError('speed_mps must be a positive finite value')
        self.world = world or HaulWorld(seed=seed + 3)
        self.speed_cmd = float(speed_mps)
        self._seed = int(seed)
        self._rng = np.random.default_rng(self._seed)

        self._t = 0.0
        self._s = 0.0  # station along route
        self._prev: VehicleState | None = None

        self._speed = float(speed_mps)
        self._speed_target = float(speed_mps)
        self._yaw = 0.0
        self._yaw_rate = 0.0
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._pitch = 0.0
        self._roll = 0.0
        self._vx = self._speed * math.cos(self._yaw)
        self._vy = self._speed * math.sin(self._yaw)
        self._vz = 0.0

        self._next_speed_s = float(self._rng.uniform(10.0, 18.0))
        self._next_bump_s = float(self._rng.uniform(8.0, 14.0))
        self._bump_amp = 0.0
        self._bump_t0 = -1.0
        self._last_ax = 0.0
        self._last_ay = 0.0
        self._last_az = 0.0

        # snap to route start
        self._x, self._y, self._yaw = self.world.centerline(0.0)
        self._z = float(self.world.height(self._x, self._y))
        self._vx = self._speed * math.cos(self._yaw)
        self._vy = self._speed * math.sin(self._yaw)

    def reset(self, t0: float = 0.0) -> None:
        self._rng = np.random.default_rng(self._seed)
        self._t = t0
        self._s = 0.0
        self._prev = None
        self._speed = self.speed_cmd
        self._speed_target = self.speed_cmd
        self._x, self._y, self._yaw = self.world.centerline(0.0)
        self._z = float(self.world.height(self._x, self._y))
        self._pitch = 0.0
        self._roll = 0.0
        self._vx = self._speed * math.cos(self._yaw)
        self._vy = self._speed * math.sin(self._yaw)
        self._vz = 0.0
        self._yaw_rate = 0.0
        self._next_speed_s = t0 + float(self._rng.uniform(10.0, 18.0))
        self._next_bump_s = t0 + float(self._rng.uniform(8.0, 14.0))
        self._bump_amp = 0.0
        self._bump_t0 = -1.0
        self._last_ax = self._last_ay = self._last_az = 0.0

    def _step_once(self, t: float, dt: float) -> None:
        # Grade-aware speed target
        slope = self.world.slope_along_heading(self._x, self._y, self._yaw)
        grade_factor = float(np.clip(1.0 - 1.8 * slope, 0.55, 1.15))
        if t >= self._next_speed_s:
            self._speed_target = float(
                np.clip(
                    self.speed_cmd * grade_factor + self._rng.normal(0.0, 0.35),
                    4.0,
                    self.speed_cmd + 1.5,
                )
            )
            self._next_speed_s = t + float(self._rng.uniform(9.0, 18.0))
        else:
            self._speed_target = float(
                np.clip(self.speed_cmd * grade_factor, 4.0, self.speed_cmd + 1.2)
            )

        speed_prev = self._speed
        x_prev, y_prev = self._x, self._y
        self._speed = _exp_smooth(self._speed, self._speed_target, dt, tau=2.0)

        # The centerline parameter is not exact arc length. Scale its advance
        # by the local derivative so world displacement matches speed_mps.
        c0x, c0y, _ = self.world.centerline(self._s)
        c1x, c1y, _ = self.world.centerline(self._s + 0.25)
        metres_per_param = max(math.hypot(c1x - c0x, c1y - c0y) / 0.25, 1e-3)
        self._s += self._speed * dt / metres_per_param
        cx, cy, path_yaw = self.world.centerline(self._s)
        _, _, yaw_ahead = self.world.centerline(self._s + 8.0)
        dyaw = _wrap_pi(yaw_ahead - path_yaw)
        yaw_rate_cmd = dyaw / 8.0 * self._speed

        self._yaw_rate = _exp_smooth(self._yaw_rate, yaw_rate_cmd, dt, tau=0.5)
        yaw_alpha = 1.0 - math.exp(-dt / 0.35)
        self._yaw = _wrap_pi(self._yaw + yaw_alpha * _wrap_pi(path_yaw - self._yaw))

        lat_noise = 0.15 * math.sin(0.05 * self._s)
        nx, ny = -math.sin(path_yaw), math.cos(path_yaw)
        self._x = cx + nx * lat_noise
        self._y = cy + ny * lat_noise
        self._vx = (self._x - x_prev) / dt
        self._vy = (self._y - y_prev) / dt

        z_road = float(self.world.height(self._x, self._y))
        bump_az = 0.0
        if t >= self._next_bump_s:
            self._bump_t0 = t
            self._bump_amp = float(self._rng.uniform(0.6, 1.8))
            self._next_bump_s = t + float(self._rng.uniform(7.0, 16.0))
        if self._bump_t0 >= 0.0:
            age = t - self._bump_t0
            if age < 0.4:
                bump_az = self._bump_amp * math.sin(math.pi * age / 0.4)
            else:
                self._bump_t0 = -1.0

        z_err = z_road - self._z
        az_spring = 8.0 * z_err - 4.5 * self._vz + bump_az
        self._vz += az_spring * dt
        self._z += self._vz * dt

        pitch_tgt = float(np.clip(-slope * 0.95, -0.18, 0.18))
        self._pitch = _exp_smooth(self._pitch, pitch_tgt, dt, tau=0.45)

        ay_body = self._speed * self._yaw_rate
        roll_tgt = float(np.clip(-0.035 * ay_body, -0.08, 0.08))
        self._roll = _exp_smooth(self._roll, roll_tgt, dt, tau=0.5)

        self._last_ax = float(np.clip((self._speed - speed_prev) / dt, -1.2, 0.9))
        self._last_ay = float(ay_body)
        az_body = az_spring * math.cos(self._pitch) * math.cos(self._roll)
        self._last_az = float(np.clip(az_body, -4.0, 4.0))

    def step(self, t: float) -> VehicleState:
        # Sub-step so large jumps (tests / hitch) stay stable
        dt_total = max(0.0, t - self._t)
        if dt_total <= 0.0 and self._prev is not None:
            return self._prev

        pitch_prev = self._pitch
        roll_prev = self._roll
        self._last_ax = self._last_ay = self._last_az = 0.0

        t_cursor = self._t
        while t_cursor < t - 1e-12:
            dt = min(0.05, t - t_cursor)
            t_cursor += dt
            self._step_once(t_cursor, dt)

        self._t = t
        dt_eff = max(dt_total, 1e-3)
        pitch_rate = (self._pitch - pitch_prev) / dt_eff
        roll_rate = (self._roll - roll_prev) / dt_eff

        st = VehicleState(
            t=t,
            x=self._x,
            y=self._y,
            z=self._z,
            yaw=self._yaw,
            pitch=self._pitch,
            roll=self._roll,
            vx=self._vx,
            vy=self._vy,
            vz=self._vz,
            yaw_rate=self._yaw_rate,
            pitch_rate=pitch_rate,
            roll_rate=roll_rate,
            ax=self._last_ax,
            ay=self._last_ay,
            az=self._last_az,
        )
        self._prev = st
        return st

    def R_body_from_enu(self, st: VehicleState) -> np.ndarray:
        cy, sy = math.cos(st.yaw), math.sin(st.yaw)
        cp, sp = math.cos(st.pitch), math.sin(st.pitch)
        cr, sr = math.cos(st.roll), math.sin(st.roll)
        Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
        return Rz @ Ry @ Rx

    def body_to_enu(self, st: VehicleState, p_body: np.ndarray) -> np.ndarray:
        R = self.R_body_from_enu(st)
        origin = np.array([st.x, st.y, st.z], dtype=float)
        return origin + R @ np.asarray(p_body, dtype=float)

    def enu_to_body(self, st: VehicleState, p_enu: np.ndarray) -> np.ndarray:
        R = self.R_body_from_enu(st)
        origin = np.array([st.x, st.y, st.z], dtype=float)
        return R.T @ (np.asarray(p_enu, dtype=float) - origin)
