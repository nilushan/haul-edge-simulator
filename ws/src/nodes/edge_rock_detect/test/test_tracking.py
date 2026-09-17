import math

import pytest

from edge_perception.tracking import Pose2D, SpatialDeduplicator, body_to_map


def test_body_to_map_uses_vehicle_translation_and_yaw():
    x, y, z = body_to_map(2.0, 0.0, 1.0, Pose2D(10.0, 20.0, 3.0, math.pi / 2))
    assert x == pytest.approx(10.0)
    assert y == pytest.approx(22.0)
    assert z == pytest.approx(4.0)


def test_spatial_dedup_is_bounded_and_expires():
    dedup = SpatialDeduplicator(cell_m=2.0, ttl_s=5.0, capacity=2)
    assert not dedup.seen_recently(0.1, 0.1, 0.0)
    assert dedup.seen_recently(1.9, 1.9, 1.0)
    assert not dedup.seen_recently(3.0, 0.0, 2.0)
    assert not dedup.seen_recently(5.0, 0.0, 3.0)
    assert len(dedup) == 2
    assert not dedup.seen_recently(0.1, 0.1, 10.0)
