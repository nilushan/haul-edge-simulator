from types import SimpleNamespace

from edge_bund_detect.alerting import CooldownGate, matching_detection


def test_cooldown_uses_elapsed_monotonic_time():
    gate = CooldownGate(2.0)
    assert gate.allow(10.0)
    assert not gate.allow(11.9)
    assert gate.allow(12.0)


def test_matching_detection_returns_only_requested_side():
    left = SimpleNamespace(details={'side': 'left'})
    right = SimpleNamespace(details={'side': 'right'})
    assert matching_detection([left, right], 'right') is right
    assert matching_detection([left], 'right') is None
