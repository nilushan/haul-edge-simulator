"""GNSS model: local ENU → WGS84 lat/lon/alt with noise and dropouts."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .vehicle import VehicleState


@dataclass
class GnssSample:
    t: float
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    h_acc_m: float
    v_acc_m: float
    fix_ok: bool
    status: int  # 0 no fix, 1 fix, 2 SBAS-ish


class GnssSimulator:
    """
    Flat-earth ENU around a fixed origin (default near a generic mine latband).
    """

    def __init__(
        self,
        origin_lat_deg: float = -22.5,
        origin_lon_deg: float = 119.8,
        origin_alt_m: float = 450.0,
        h_noise_m: float = 1.5,
        v_noise_m: float = 3.0,
        dropout_prob: float = 0.01,
        seed: int = 13,
    ) -> None:
        if not -90.0 < origin_lat_deg < 90.0:
            raise ValueError('origin_lat_deg must be strictly between -90 and 90')
        if h_noise_m < 0 or v_noise_m < 0:
            raise ValueError('GNSS noise values cannot be negative')
        if not 0.0 <= dropout_prob <= 1.0:
            raise ValueError('dropout_prob must be between 0 and 1')
        self.lat0 = math.radians(origin_lat_deg)
        self.lon0 = math.radians(origin_lon_deg)
        self.alt0 = float(origin_alt_m)
        self.h_noise = float(h_noise_m)
        self.v_noise = float(v_noise_m)
        self.dropout_prob = float(dropout_prob)
        self._rng = np.random.default_rng(seed)
        # metres per radian
        self._R = 6378137.0
        self._fallback_fix = (float(origin_lat_deg), float(origin_lon_deg), self.alt0)
        self._last_fix: tuple[float, float, float] | None = None

    def sample(self, st: VehicleState) -> GnssSample:
        if self._rng.random() < self.dropout_prob:
            lat, lon, alt = self._last_fix or self._fallback_fix
            return GnssSample(
                t=st.t,
                latitude_deg=lat,
                longitude_deg=lon,
                altitude_m=alt,
                h_acc_m=50.0,
                v_acc_m=50.0,
                fix_ok=False,
                status=0,
            )

        # ENU → geodetic (small-offset approximation)
        d_north = st.y + float(self._rng.normal(0.0, self.h_noise))
        d_east = st.x + float(self._rng.normal(0.0, self.h_noise))
        d_up = st.z + float(self._rng.normal(0.0, self.v_noise))

        lat = self.lat0 + d_north / self._R
        lon = self.lon0 + d_east / (self._R * math.cos(self.lat0))
        alt = self.alt0 + d_up
        self._last_fix = (math.degrees(lat), math.degrees(lon), alt)

        return GnssSample(
            t=st.t,
            latitude_deg=math.degrees(lat),
            longitude_deg=math.degrees(lon),
            altitude_m=alt,
            h_acc_m=self.h_noise * 2.0,
            v_acc_m=self.v_noise * 2.0,
            fix_ok=True,
            status=2,
        )
