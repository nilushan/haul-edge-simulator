from edge_bund_detect.alerting import CooldownGate


def test_cooldown_uses_elapsed_monotonic_time():
    gate = CooldownGate(2.0)
    assert gate.allow(10.0)
    assert not gate.allow(11.9)
    assert gate.allow(12.0)
