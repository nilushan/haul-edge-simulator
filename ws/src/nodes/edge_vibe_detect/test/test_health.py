import pytest

from edge_vibe_detect.health import is_fresh, validate_thresholds


def test_threshold_validation_rejects_inverted_ranges():
    with pytest.raises(ValueError):
        validate_thresholds(2.0, 1.0, 1.0, 2.0, 0.0)
    with pytest.raises(ValueError):
        validate_thresholds(1.0, 2.0, 3.0, 2.0, 0.0)


def test_odom_freshness_handles_missing_stale_and_clock_regression():
    assert not is_fresh(None, 10.0, 1.0)
    assert is_fresh(9.5, 10.0, 1.0)
    assert not is_fresh(8.0, 10.0, 1.0)
    assert not is_fresh(11.0, 10.0, 1.0)
