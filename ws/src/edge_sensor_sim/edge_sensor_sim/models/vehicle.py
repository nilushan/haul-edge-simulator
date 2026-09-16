"""Shared haul-truck motion truth used by all sensor models."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


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
    """First-order lag toward target (realistic plant response)."""
    if tau <= 1e-6:
        return target
    a = 1.0 - math.exp(-dt / tau)
    return prev + a * (target - prev)


class VehicleSimulator:
    """
    Haul-truck motion intended to look plant-like, not toy-cyclic:

      - mostly steady cruise speed with rare gentle accel/brake
      - long straight segments + infrequent slow heading changes
      - nearly flat grade; pitch/roll are small and filtered
      - irregular (not periodic) road bumps
    """

    def __init__(
        self,
        speed_mps: float = 7.5,
        seed: int = 7,
    ) -> None:
        self.speed_cmd = float(speed_mps)
        self._rng = np.random.default_rng(seed)

        self._t = 0.0
        self._s = 0.0
        self._prev: VehicleState | None = None

        # Internal plant state
        self._speed = float(speed_mps)
        self._yaw = 0.0
        self._yaw_rate = 0.0
        self._yaw_rate_cmd = 0.0
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._pitch = 0.0
        self._roll = 0.0
        self._vz = 0.0

        # Event clocks (irregular)
        self._next_turn_s = float(self._rng.uniform(18.0, 28.0))
        self._turn_until_s = -1.0
        self._next_speed_s = float(self._rng.uniform(12.0, 22.0))
        self._speed_target = float(speed_mps)
        self._next_bump_s = float(self._rng.uniform(9.0, 16.0))
        self._bump_amp = 0.0
        self._bump_t0 = -1.0

    def reset(self, t0: float = 0.0) -> None:
        self._t = t0
        self._s = 0.0
        self._prev = None
        self._speed = self.speed_cmd
        self._speed_target = self.speed_cmd
        self._yaw = 0.0
        self._yaw_rate = 0.0
        self._yaw_rate_cmd = 0.0
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._pitch = 0.0
        self._roll = 0.0
        self._vz = 0.0
        self._next_turn_s = float(self._rng.uniform(18.0, 28.0))
        self._turn_until_s = -1.0
        self._next_speed_s = float(self._rng.uniform(12.0, 22.0))
        self._next_bump_s = float(self._rng.uniform(9.0, 16.0))
        self._bump_amp = 0.0
        self._bump_t0 = -1.0

    def _schedule_events(self, t: float) -> None:
        # Occasional slow turn (not continuous curve)
        if t >= self._next_turn_s and t >= self._turn_until_s:
            duration = float(self._rng.uniform(4.0, 8.0))
            # gentle yaw rate ~ 2–5 deg/s
            sign = 1.0 if self._rng.random() < 0.5 else -1.0
            self._yaw_rate_cmd = sign * math.radians(float(self._rng.uniform(2.0, 5.0)))
            self._turn_until_s = t + duration
            self._next_turn_s = t + duration + float(self._rng.uniform(16.0, 30.0))
        if t >= self._turn_until_s:
            self._yaw_rate_cmd = 0.0

        # Rare cruise speed change
        if t >= self._next_speed_s:
            self._speed_target = float(
                np.clip(
                    self.speed_cmd + self._rng.normal(0.0, 0.6),
                    max(4.0, self.speed_cmd - 1.5),
                    self.speed_cmd + 1.2,
                )
            )
            self._next_speed_s = t + float(self._rng.uniform(14.0, 26.0))

        # Irregular bumps (not periodic)
        if t >= self._next_bump_s:
            self._bump_t0 = t
            self._bump_amp = float(self._rng.uniform(0.8, 2.2))  # m/s² pulse, mild
            self._next_bump_s = t + float(self._rng.uniform(11.0, 22.0))

    def step(self, t: float) -> VehicleState:
        """Advance absolute simulation time to t (seconds)."""
        dt = max(1e-4, t - self._t)
        # Cap dt so large jumps (loop reset handled externally) stay stable
        dt = min(dt, 0.05)
        self._t = t
        self._schedule_events(t)

        # Speed plant
        self._speed = _exp_smooth(self._speed, self._speed_target, dt, tau=2.5)
        ax_cmd = (self._speed_target - self._speed) / 2.5  # rough

        # Yaw plant — lag commanded rate
        self._yaw_rate = _exp_smooth(self._yaw_rate, self._yaw_rate_cmd, dt, tau=0.8)
        self._yaw = _wrap_pi(self._yaw + self._yaw_rate * dt)

        # Integrate planar pose
        self._x += self._speed * math.cos(self._yaw) * dt
        self._y += self._speed * math.sin(self._yaw) * dt
        self._s += self._speed * dt

        # Grade: very mild long wavelength (barely visible), not a bounce
        z_grade = 0.04 * math.sin(0.012 * self._s)  # ~ cm-level over long distance
        # Bump as short vertical accel pulse → integrate carefully
        bump_az = 0.0
        if self._bump_t0 >= 0.0:
            age = t - self._bump_t0
            if age < 0.45:
                bump_az = self._bump_amp * math.sin(math.pi * age / 0.45)
            else:
                self._bump_t0 = -1.0
                self._bump_amp = 0.0

        # Vertical dynamics: spring-ish toward grade
        z_err = z_grade - self._z
        az_spring = 3.5 * z_err - 2.8 * self._vz  # damped
        az_body = az_spring + bump_az
        self._vz += az_body * dt
        self._z += self._vz * dt

        # Pitch: follow grade slope + small lag from vertical vel (not oscillatory show)
        slope = 0.04 * 0.012 * math.cos(0.012 * self._s)  # dz/ds
        pitch_tgt = float(np.clip(slope * 0.9 + 0.015 * self._vz, -0.04, 0.04))
        pitch_prev = self._pitch
        self._pitch = _exp_smooth(self._pitch, pitch_tgt, dt, tau=0.6)
        pitch_rate = (self._pitch - pitch_prev) / dt

        # Roll: from lateral accel only (steady turn), heavily limited
        ay_body = self._speed * self._yaw_rate
        roll_tgt = float(np.clip(-0.02 * ay_body, -0.03, 0.03))  # ~1.7 deg max
        roll_prev = self._roll
        self._roll = _exp_smooth(self._roll, roll_tgt, dt, tau=0.7)
        roll_rate = (self._roll - roll_prev) / dt

        ax_body = float(np.clip(ax_cmd, -0.8, 0.6))
        # residual vertical for IMU (not huge)
        az_out = float(np.clip(az_body, -3.0, 3.0))

        st = VehicleState(
            t=t,
            x=self._x,
            y=self._y,
            z=self._z,
            yaw=self._yaw,
            pitch=self._pitch,
            roll=self._roll,
            vx=self._speed * math.cos(self._yaw),
            vy=self._speed * math.sin(self._yaw),
            vz=self._vz,
            yaw_rate=self._yaw_rate,
            pitch_rate=pitch_rate,
            roll_rate=roll_rate,
            ax=ax_body,
            ay=ay_body,
            az=az_out,
        )
        self._prev = st
        return st

    def R_body_from_enu(self, st: VehicleState) -> np.ndarray:
        """R such that p_enu = R @ p_body."""
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
