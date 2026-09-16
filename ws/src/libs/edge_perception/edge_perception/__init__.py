"""Perception library: geometry, detectors, schemas (no ROS runtime)."""

from edge_perception.schema import (
    AlertEvent,
    Detection,
    DetectionSet,
    Label,
    Severity,
)

__all__ = [
    'AlertEvent',
    'Detection',
    'DetectionSet',
    'Label',
    'Severity',
]
