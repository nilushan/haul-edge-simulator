"""Stable detection / alert schemas shared by all detector nodes."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import IntEnum
from typing import Any, Dict, List, Optional
import uuid


class Label(IntEnum):
    GROUND = 0
    ROCK = 1
    BUND = 2
    OBSTACLE = 3
    UNKNOWN = 255


class Severity:
    INFO = 'info'
    WARN = 'warn'
    CRITICAL = 'critical'


@dataclass
class Detection:
    """One object hypothesis for a single frame (not necessarily alerted)."""

    type: str
    track_id: str = ''
    confidence: float = 0.0
    frame_id: str = 'base_link'
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    radius_m: float = 0.0
    extent_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    label: int = int(Label.UNKNOWN)
    point_count: int = 0
    details: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d['extent_m'] = list(self.extent_m)
        return d


@dataclass
class DetectionSet:
    t: float
    frame_id: str
    source_node: str
    detections: List[Detection] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            'schema': 'edge.detections.v1',
            't': self.t,
            'frame_id': self.frame_id,
            'source_node': self.source_node,
            'detections': [d.to_dict() for d in self.detections],
        }


@dataclass
class AlertEvent:
    """Edge-triggered product event (rocks, bund issues, vibration, …)."""

    type: str
    severity: str = Severity.WARN
    confidence: float = 0.0
    source_node: str = ''
    vehicle_id: str = 'haul-01'
    frame_id: str = 'map'
    t_ros: Optional[float] = None
    t_vehicle: float = 0.0
    event_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    lat: Optional[float] = None
    lon: Optional[float] = None
    alt: Optional[float] = None
    geometry: Dict[str, Any] = field(default_factory=dict)
    details: Dict[str, Any] = field(default_factory=dict)
    cloud_ref: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            'schema': 'edge.alert.v1',
            'event_id': self.event_id,
            't_ros': self.t_ros,
            't_vehicle': self.t_vehicle,
            'vehicle_id': self.vehicle_id,
            'source_node': self.source_node,
            'type': self.type,
            'severity': self.severity,
            'confidence': self.confidence,
            'frame_id': self.frame_id,
            'pose': {'x': self.x, 'y': self.y, 'z': self.z},
            'llh': {'lat': self.lat, 'lon': self.lon, 'alt': self.alt},
            'geometry': self.geometry,
            'details': self.details,
            'cloud_ref': self.cloud_ref,
        }
