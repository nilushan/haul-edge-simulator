"""Shared haul-truck motion truth used by all sensor models."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple

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
    ax: float  # body longitudinal accel (rough)
    ay: float
    az: float  # body vertical accel including gravity later in IMU


def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class VehicleSimulator:
    """
    Simple haul trajectory:
      - drives forward at nominal speed along a gentle curve
      - road undulation (sinusoid in z / pitch)
      - sparse bump impulses (tyre hits)
    """

    def __init__(
        self,
        speed_mps: float = 8.0,
        curve_omega: float = 0.015,
        undulation_amp_m: float = 0.12,
        undulation_freq_hz: float = 0.35,
        bump_period_s: float = 7.5,
        bump_amp_mps2: float = 4.0,
        seed: int = 7,
    ) -> None:
        self.speed = float(speed_mps)
        self.curve_omega = float(curve_omega)
        self.und_amp = float(undulation_amp_m)
        self.und_w = 2.0 * math.pi * float(undulation_freq_hz)
        self.bump_period = float(bump_period_s)
        self.bump_amp = float(bump_amp_mps2)
        self._rng = np.random.default_rng(seed)

        self._t = 0.0
        self._s = 0.0  # arc length
        self._prev: VehicleState | None = None

    def reset(self, t0: float = 0.0) -> None:
        self._t = t0
        self._s = 0.0
        self._prev = None

    def step(self, t: float) -> VehicleState:
        """Advance absolute simulation time to t (seconds)."""
        dt = max(1e-4, t - self._t)
        self._t = t
        self._s += self.speed * dt

        # Path in ENU: slow left curve
        yaw = self.curve_omega * self._s
        # Integrate position along yaw
        if self._prev is None:
            x, y = 0.0, 0.0
        else:
            x = self._prev.x + self.speed * math.cos(self._prev.yaw) * dt
            y = self._prev.y + self.speed * math.sin(self._prev.yaw) * dt

        # Road wave + discrete bumps
        z_road = self.und_amp * math.sin(self.und_w * self._s)
        pitch = self.und_amp * self.und_w * math.cos(self.und_w * self._s) * 0.5

        bump = 0.0
        phase = self._s % self.bump_period
        if phase < 0.35:
            # smooth bump pulse
            u = phase / 0.35
            bump = self.bump_amp * math.sin(math.pi * u)

        z = z_road
        vz = (z - (self._prev.z if self._prev else z)) / dt
        ax_body = 0.15 * math.sin(0.2 * t)  # mild throttle variation
        ay_body = self.speed * self.curve_omega  # centripetal in body y (approx)
        az_body = bump + (vz - (self._prev.vz if self._prev else 0.0)) / dt

        yaw_rate = self.curve_omega * self.speed
        pitch_rate = (pitch - (self._prev.pitch if self._prev else pitch)) / dt
        roll = 0.02 * math.sin(0.5 * t)
        roll_rate = (roll - (self._prev.roll if self._prev else roll)) / dt

        st = VehicleState(
            t=t,
            x=x,
            y=y,
            z=z,
            yaw=_wrap_pi(yaw),
            pitch=pitch,
            roll=roll,
            vx=self.speed * math.cos(yaw),
            vy=self.speed * math.sin(yaw),
            vz=vz,
            yaw_rate=yaw_rate,
            pitch_rate=pitch_rate,
            roll_rate=roll_rate,
            ax=ax_body,
            ay=ay_body,
            az=az_body,
        )
        self._prev = st
        return st

    def R_body_from_enu(self, st: VehicleState) -> np.ndarray:
        """Rotation matrix: body <- ENU? We use R such that p_enu = R @ p_body."""
        cy, sy = math.cos(st.yaw), math.sin(st.yaw)
        cp, sp = math.cos(st.pitch), math.sin(st.pitch)
        cr, sr = math.cos(st.roll), math.sin(st.roll)
        # ZYX yaw-pitch-roll
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
