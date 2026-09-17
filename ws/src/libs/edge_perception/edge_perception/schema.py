"""Stable detection / alert schemas shared by all detector nodes."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import IntEnum
import math
from typing import Any, Dict, List, Optional, Tuple
import uuid


class Label(IntEnum):
    """Per-point semantic class published on labelled clouds."""

    GROUND = 0      # traversable surface outside the drivable corridor
    ROCK = 1
    BUND = 2        # berm crest at or above the compliance height
    OBSTACLE = 3    # non-ground return that is not a rock or a bund
    ROAD = 4        # drivable corridor surface
    BUND_LOW = 5    # berm crest below the compliance height
    UNKNOWN = 255


class Severity:
    INFO = 'info'
    WARN = 'warn'
    CRITICAL = 'critical'


SEVERITY_COLOR: Dict[str, str] = {
    Severity.INFO: '#4aa3ff',
    Severity.WARN: '#e6b450',
    Severity.CRITICAL: '#f07178',
}


@dataclass(frozen=True)
class ClassStyle:
    """How one point class is named and drawn everywhere in the stack."""

    label: int
    key: str
    title: str
    color: str
    layer: str  # viz toggle group this class belongs to
    description: str = ''


@dataclass(frozen=True)
class LayerStyle:
    """A viz toggle group covering one or more point classes."""

    key: str
    title: str
    default_on: bool = True


@dataclass(frozen=True)
class EventStyle:
    """How one alert type is named and drawn on the map."""

    type: str
    title: str
    short: str
    color: str
    description: str = ''


CLASS_STYLES: Tuple[ClassStyle, ...] = (
    ClassStyle(
        label=int(Label.ROAD),
        key='road',
        title='Road',
        color='#4c7fb8',
        layer='ground',
        description='Drivable corridor surface.',
    ),
    ClassStyle(
        label=int(Label.GROUND),
        key='ground',
        title='Ground',
        color='#6f6a45',
        layer='ground',
        description='Traversable surface outside the corridor.',
    ),
    ClassStyle(
        label=int(Label.BUND),
        key='bund',
        title='Bund',
        color='#2fd4c6',
        layer='bund',
        description='Berm crest at or above the compliance height.',
    ),
    ClassStyle(
        label=int(Label.BUND_LOW),
        key='bund_low',
        title='Bund low',
        color='#ff4d6d',
        layer='bund',
        description='Berm crest below the compliance height.',
    ),
    ClassStyle(
        label=int(Label.ROCK),
        key='rock',
        title='Rock',
        color='#ff7a1a',
        layer='rock',
        description='Compact non-ground cluster of rock-like size.',
    ),
    ClassStyle(
        label=int(Label.OBSTACLE),
        key='obstacle',
        title='Non-ground',
        color='#c084fc',
        layer='obstacle',
        description='Non-ground return that is not a rock or a bund.',
    ),
    ClassStyle(
        label=int(Label.UNKNOWN),
        key='unknown',
        title='Unclassified',
        color='#7d8899',
        layer='unknown',
        description='No ground support in the cell, so no class was assigned.',
    ),
)

LAYER_STYLES: Tuple[LayerStyle, ...] = (
    LayerStyle(key='ground', title='road / ground', default_on=True),
    LayerStyle(key='bund', title='bunds', default_on=True),
    LayerStyle(key='rock', title='rocks', default_on=True),
    LayerStyle(key='obstacle', title='non-ground', default_on=True),
    LayerStyle(key='unknown', title='unclassified', default_on=False),
)

EVENT_STYLES: Tuple[EventStyle, ...] = (
    EventStyle(
        type='rock',
        title='Rock on road',
        short='ROCK',
        color='#ff7a1a',
        description='Rock-sized obstruction in or beside the driving lane.',
    ),
    EventStyle(
        type='bund_low',
        title='Bund height below spec',
        short='BUND LOW',
        color='#ff4d6d',
        description='Berm crest measured below the compliance height.',
    ),
    EventStyle(
        type='bund_gap',
        title='Bund gap',
        short='BUND GAP',
        color='#e84cff',
        description='No berm crest over a stretch of observed shoulder.',
    ),
    EventStyle(
        type='excessive_vibration',
        title='Excessive vibration',
        short='VIBE',
        color='#ffd166',
        description='Road roughness above the ride-quality threshold.',
    ),
    EventStyle(
        type='obstacle',
        title='Obstacle',
        short='OBSTACLE',
        color='#c084fc',
        description='Non-ground obstruction that is not classified as a rock.',
    ),
)

CLASS_BY_LABEL: Dict[int, ClassStyle] = {c.label: c for c in CLASS_STYLES}
CLASS_BY_KEY: Dict[str, ClassStyle] = {c.key: c for c in CLASS_STYLES}
EVENT_BY_TYPE: Dict[str, EventStyle] = {e.type: e for e in EVENT_STYLES}


def class_style(label: int) -> ClassStyle:
    """Style for a point label, falling back to the unclassified style."""
    return CLASS_BY_LABEL.get(int(label), CLASS_BY_LABEL[int(Label.UNKNOWN)])


def event_style(event_type: str) -> Optional[EventStyle]:
    return EVENT_BY_TYPE.get(str(event_type))


def style_catalog() -> Dict[str, Any]:
    """JSON-ready class/event style table shared with the browser."""
    return {
        'schema': 'edge.styles.v1',
        'classes': [asdict(c) for c in CLASS_STYLES],
        'layers': [asdict(layer) for layer in LAYER_STYLES],
        'events': [asdict(e) for e in EVENT_STYLES],
        'severities': dict(SEVERITY_COLOR),
    }


def json_safe(value: Any) -> Any:
    """
    Replace non-finite floats with ``None`` throughout a payload.

    ``json.dumps`` happily writes bare ``NaN``/``Infinity``, which no strict
    JSON reader accepts — a browser tick carrying one is dropped whole. Every
    schema leaving this module is sanitized here instead.
    """
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


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
        return json_safe(d)


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
    # Short human-readable tag drawn next to the event on the map.
    label: str = ''
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
        return json_safe({
            'schema': 'edge.alert.v1',
            'event_id': self.event_id,
            't_ros': self.t_ros,
            't_vehicle': self.t_vehicle,
            'vehicle_id': self.vehicle_id,
            'source_node': self.source_node,
            'type': self.type,
            'severity': self.severity,
            'confidence': self.confidence,
            'label': self.label or (event_style(self.type).short if event_style(self.type) else self.type),
            'frame_id': self.frame_id,
            'pose': {'x': self.x, 'y': self.y, 'z': self.z},
            'llh': {'lat': self.lat, 'lon': self.lon, 'alt': self.alt},
            'geometry': self.geometry,
            'details': self.details,
            'cloud_ref': self.cloud_ref,
        })
