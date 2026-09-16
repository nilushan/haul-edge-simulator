"""IMU model: body rates + specific force with bias and noise."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .vehicle import VehicleState


@dataclass
class ImuSample:
    t: float
    # angular velocity [rad/s] in body frame
    gx: float
    gy: float
    gz: float
    # linear acceleration [m/s^2] in body frame (specific force-ish: includes g)
    ax: float
    ay: float
    az: float


class ImuSimulator:
    def __init__(
        self,
        gyro_noise_std: float = 0.002,
        accel_noise_std: float = 0.05,
        gyro_bias: tuple[float, float, float] = (0.001, -0.0005, 0.0002),
        accel_bias: tuple[float, float, float] = (0.02, -0.01, 0.03),
        gravity: float = 9.80665,
        seed: int = 11,
    ) -> None:
        self.gyro_noise_std = float(gyro_noise_std)
        self.accel_noise_std = float(accel_noise_std)
        self.gyro_bias = np.asarray(gyro_bias, dtype=float)
        self.accel_bias = np.asarray(accel_bias, dtype=float)
        self.g = float(gravity)
        self._rng = np.random.default_rng(seed)

    def sample(self, st: VehicleState) -> ImuSample:
        # Angular rate from vehicle truth
        w = np.array([st.roll_rate, st.pitch_rate, st.yaw_rate], dtype=float)
        w = w + self.gyro_bias + self._rng.normal(0.0, self.gyro_noise_std, size=3)

        # Specific force: body accel - R^T * g_enu  (accelerometer measures reaction to gravity)
        # g_enu = [0,0,-g] if z up; accelerometer at rest reads +g on z if z up body.
        # Using ENU z-up: gravity vector in ENU is (0,0,-g).
        # a_meas ≈ a_body_nav - R_body_from_enu.T @ g_enu
        # For small angles, approximate gravity in body:
        # az ≈ st.az + g * cos(pitch)*cos(roll) ...
        cp, sp = np.cos(st.pitch), np.sin(st.pitch)
        cr, sr = np.cos(st.roll), np.sin(st.roll)
        # gravity expressed in body (Z-up vehicle)
        g_body = np.array([
            -sp * self.g,
            sr * cp * self.g,
            cr * cp * self.g,
        ], dtype=float)
        a_nav_body = np.array([st.ax, st.ay, st.az], dtype=float)
        a = a_nav_body + g_body + self.accel_bias
        a = a + self._rng.normal(0.0, self.accel_noise_std, size=3)

        return ImuSample(
            t=st.t,
            gx=float(w[0]),
            gy=float(w[1]),
            gz=float(w[2]),
            ax=float(a[0]),
            ay=float(a[1]),
            az=float(a[2]),
        )
