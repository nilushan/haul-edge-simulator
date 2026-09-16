"""Pure validation and freshness helpers for the vibration node."""

from __future__ import annotations

import math
from typing import Optional


def validate_thresholds(
    rms_warn: float,
    rms_crit: float,
    peak_warn: float,
    peak_crit: float,
    min_speed_mps: float,
) -> None:
    values = (rms_warn, rms_crit, peak_warn, peak_crit, min_speed_mps)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('vibration thresholds must be finite')
    if not 0.0 <= rms_warn <= rms_crit:
        raise ValueError('require 0 <= rms_warn <= rms_crit')
    if not 0.0 <= peak_warn <= peak_crit:
        raise ValueError('require 0 <= peak_warn <= peak_crit')
    if min_speed_mps < 0:
        raise ValueError('min_speed_mps cannot be negative')


def is_fresh(last_received_s: Optional[float], now_s: float, timeout_s: float) -> bool:
    return (
        last_received_s is not None
        and timeout_s > 0
        and 0.0 <= now_s - last_received_s <= timeout_s
    )
