"""Pure bund-alert integration helpers."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class CooldownGate:
    interval_s: float
    last_s: float = float('-inf')

    def __post_init__(self) -> None:
        if self.interval_s < 0:
            raise ValueError('cooldown interval cannot be negative')

    def allow(self, now_s: float) -> bool:
        if now_s - self.last_s < self.interval_s:
            return False
        self.last_s = now_s
        return True
