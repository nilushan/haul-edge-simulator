"""
Whole-frame semantic classification for one LiDAR scan.

Turns a body-frame point cloud into per-point classes (road, ground, bund,
bund-low, rock, non-ground) plus the object detections and product events those
classes imply. Shared by the ROS detector nodes and the inline viz pipeline so
both paint the same taxonomy.

Body frame is ROS convention: +X forward, +Y left, +Z up.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from edge_perception.ground import estimate_ground_grid
from edge_perception.rocks import RockParams, detect_rocks
from edge_perception.schema import (
    AlertEvent,
    Detection,
    Label,
    Severity,
    class_style,
    event_style,
)


@dataclass
class SemanticParams:
    """Thresholds for the frame classifier (metres unless noted)."""

    ground_cell_m: float = 0.75
    ground_band_m: float = 0.20
    # Fallback corridor width when the crest estimate has too little support.
    road_half_width_m: float = 6.5
    road_margin_m: float = 0.6
    shoulder_band_m: float = 3.5
    x_bin_m: float = 2.0
    min_bin_points: int = 2
    # Design height of a compliant bund and the height below which it is an
    # issue. Mine standard is typically ~2/3 of the largest tyre diameter.
    bund_spec_height_m: float = 1.65
    bund_min_height_m: float = 1.10
    # Below this the shoulder is treated as having no bund at all.
    bund_present_min_m: float = 0.35
    gap_length_m: float = 10.0
    min_shoulder_span_m: float = 12.0
    # Beyond this range the straight-corridor model stops holding on curves.
    corridor_max_x_m: float = 30.0
    # A crest is only measurable when the shoulder was sampled this far past
    # it; otherwise the beam grazed the inner flank and would read low.
    crest_observed_beyond_m: float = 0.6
    # An unsampled shoulder only counts as absent within this range.
    gap_observed_max_x_m: float = 22.0
    obstacle_min_hag_m: float = 0.25
    # Ceiling on points handed to the clusterer, so a dense frame cannot
    # stall an edge node or the viz tick pump.
    max_cluster_points: int = 2000
    event_min_range_m: float = 6.0
    event_max_range_m: float = 30.0
    rock: RockParams = field(default_factory=RockParams)


@dataclass
class BundSegment:
    """One contiguous run of shoulder bins sharing a bund state."""

    side: str  # 'left' | 'right'
    state: str  # 'ok' | 'low' | 'missing'
    x_start: float
    x_end: float
    y: float
    z: float
    min_height_m: float  # NaN when the segment has no measured crest
    mean_height_m: float
    bins: int

    @property
    def length_m(self) -> float:
        return float(self.x_end - self.x_start)


@dataclass
class FrameSemantics:
    """Per-point classes plus the objects and events derived from them."""

    labels: np.ndarray  # (N,) int16, aligned with the input points
    conf: np.ndarray  # (N,) float32
    hag: np.ndarray  # (N,) float32, NaN where no ground support
    road_half_width_m: float
    detections: List[Detection] = field(default_factory=list)
    events: List[AlertEvent] = field(default_factory=list)
    segments: List[BundSegment] = field(default_factory=list)
    counts: Dict[str, int] = field(default_factory=dict)


def _side_sign(side: str) -> float:
    return 1.0 if side == 'left' else -1.0


def estimate_road_half_width(
    points: np.ndarray,
    params: SemanticParams,
) -> float:
    """
    Lateral half-width of the drivable corridor, measured from the berm toes.

    Per longitudinal bin the road surface is referenced from the lane centre,
    then the innermost clearly raised return on each side marks the berm toe.
    Taking a percentile of the per-bin toes keeps a few rocks near the shoulder
    from pulling the corridor in. Falls back to the configured width when too
    few bins carry a toe.
    """
    p = params
    finite = np.all(np.isfinite(points), axis=1)
    finite &= np.abs(points[:, 0]) <= p.corridor_max_x_m
    if not np.any(finite):
        return float(p.road_half_width_m)

    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    idx = np.flatnonzero(finite)
    bins = np.floor(x[idx] / p.x_bin_m).astype(np.int64)
    centre_lane = np.abs(y[idx]) <= 2.5

    toes: List[float] = []
    for b in np.unique(bins):
        in_bin = bins == b
        lane = idx[in_bin & centre_lane]
        if lane.size < p.min_bin_points:
            continue
        reference = float(np.percentile(z[lane], 20.0))
        raised = idx[in_bin & ~centre_lane]
        raised = raised[(z[raised] - reference) >= p.bund_present_min_m]
        raised = raised[np.abs(y[raised]) <= 16.0]
        if raised.size:
            toes.append(float(np.min(np.abs(y[raised]))))
    if len(toes) < 3:
        return float(p.road_half_width_m)
    # The toe is where the berm starts rising, which is exactly the edge of
    # the drivable surface — no further margin is applied here.
    estimate = float(np.percentile(toes, 60.0))
    if not np.isfinite(estimate):
        return float(p.road_half_width_m)
    return float(np.clip(estimate, 3.0, 12.0))


def _bund_segments(
    points: np.ndarray,
    hag: np.ndarray,
    labels: np.ndarray,
    conf: np.ndarray,
    road_hw: float,
    params: SemanticParams,
) -> List[BundSegment]:
    """
    Classify each shoulder bin, label its points, and group runs of bins.

    Crest height is measured against the road surface in the same longitudinal
    bin, not against the local ground grid: inside a berm the grid estimate
    rides up with the berm itself and would report a crest of a few centimetres.
    """
    p = params
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    finite = np.all(np.isfinite(points), axis=1)
    # The straight-corridor model only holds near the vehicle; beyond that the
    # haul road curves out of the body-frame band.
    finite &= np.abs(x) <= p.corridor_max_x_m
    if not np.any(finite):
        return []

    inner = road_hw - p.road_margin_m
    outer = road_hw + p.shoulder_band_m
    x_min = float(np.min(x[finite]))
    road_seen = finite & (np.abs(y) <= road_hw * 0.75)
    if not np.any(road_seen):
        return []

    def bin_of(values: np.ndarray) -> np.ndarray:
        return np.floor((values - x_min) / p.x_bin_m).astype(np.int64)

    road_idx = np.flatnonzero(road_seen)
    road_bins = bin_of(x[road_idx])
    road_z: Dict[int, float] = {}
    for b in np.unique(road_bins):
        sel = road_idx[road_bins == b]
        if sel.size >= p.min_bin_points:
            road_z[int(b)] = float(np.percentile(z[sel], 20.0))
    if not road_z:
        return []

    segments: List[BundSegment] = []
    for side in ('left', 'right'):
        sign = _side_sign(side)
        lateral = sign * y
        shoulder_idx = np.flatnonzero(finite & (lateral >= inner) & (lateral <= outer))
        shoulder_bins = bin_of(x[shoulder_idx]) if shoulder_idx.size else np.zeros((0,), dtype=np.int64)

        states: Dict[int, Tuple[str, float, float, float]] = {}
        for b in sorted(road_z):
            reference = road_z[b]
            sel = shoulder_idx[shoulder_bins == b] if shoulder_idx.size else np.zeros((0,), dtype=np.int64)
            if sel.size < p.min_bin_points:
                bin_x = x_min + (b + 0.5) * p.x_bin_m
                observed = abs(float(bin_x)) <= p.gap_observed_max_x_m
                states[b] = (
                    'missing' if observed else 'unknown',
                    float('nan'),
                    sign * (road_hw + 1.0),
                    reference,
                )
                continue
            crest = sel[int(np.argmax(z[sel]))]
            height = float(z[crest] - reference)
            # Only the raised part of the shoulder is bund; its apron is ground.
            raised = sel[z[sel] - reference > 0.5 * p.bund_present_min_m]
            measurable = (
                float(np.max(lateral[sel]) - lateral[crest]) >= p.crest_observed_beyond_m
            )

            if height < p.bund_present_min_m:
                absent = measurable or abs(float(x[crest])) <= p.gap_observed_max_x_m
                states[b] = (
                    'missing' if absent else 'unknown',
                    height,
                    float(y[crest]),
                    float(z[crest]),
                )
                continue
            if not measurable:
                # Structure is there but its top was never sampled: show it as a
                # bund, and raise no height finding we cannot stand behind.
                states[b] = ('unknown', float('nan'), float(y[crest]), float(z[crest]))
                labels[raised] = int(Label.BUND)
                conf[raised] = 0.35
                continue

            state = 'ok' if height >= p.bund_min_height_m else 'low'
            states[b] = (state, height, float(y[crest]), float(z[crest]))
            label = Label.BUND if state == 'ok' else Label.BUND_LOW
            labels[raised] = int(label)
            if state == 'ok':
                conf[raised] = float(np.clip(height / max(p.bund_spec_height_m, 1e-3), 0.4, 1.0))
            else:
                deficit = (p.bund_min_height_m - height) / max(p.bund_min_height_m, 1e-3)
                conf[raised] = float(np.clip(0.55 + 0.45 * deficit, 0.0, 1.0))

        ordered = sorted(states)
        run_start: Optional[int] = None
        for position, b in enumerate(ordered):
            state = states[b][0]
            if run_start is None:
                run_start = b
            is_last = position == len(ordered) - 1
            contiguous = (
                not is_last
                and ordered[position + 1] == b + 1
                and states[ordered[position + 1]][0] == state
            )
            if contiguous:
                continue
            run = [k for k in ordered if run_start <= k <= b]
            heights = np.asarray([states[k][1] for k in run], dtype=float)
            known = heights[np.isfinite(heights)]
            ys = np.asarray([states[k][2] for k in run], dtype=float)
            zs = np.asarray([states[k][3] for k in run], dtype=float)
            segments.append(
                BundSegment(
                    side=side,
                    state=state,
                    x_start=x_min + run_start * p.x_bin_m,
                    x_end=x_min + (b + 1) * p.x_bin_m,
                    y=float(np.mean(ys)),
                    z=float(np.mean(zs)),
                    min_height_m=float(np.min(known)) if known.size else float('nan'),
                    mean_height_m=float(np.mean(known)) if known.size else float('nan'),
                    bins=len(run),
                )
            )
            run_start = None

    return segments


def _rock_label_points(
    points: np.ndarray,
    labels: np.ndarray,
    conf: np.ndarray,
    candidate_idx: np.ndarray,
    detections: List[Detection],
) -> None:
    if not detections or candidate_idx.size == 0:
        return
    candidates = points[candidate_idx]
    for d in detections:
        centre = np.array([d.x, d.y, d.z], dtype=float)
        radius = max(d.radius_m * 1.25, 0.35)
        near = np.linalg.norm(candidates - centre, axis=1) <= radius
        if not np.any(near):
            continue
        selected = candidate_idx[near]
        labels[selected] = int(Label.ROCK)
        conf[selected] = float(np.clip(d.confidence, 0.0, 1.0))


def _event_label(event_type: str, detail: str) -> str:
    style = event_style(event_type)
    head = style.short if style else event_type.upper()
    return f'{head} · {detail}' if detail else head


def _rock_events(
    detections: List[Detection],
    params: SemanticParams,
    *,
    t: float,
    frame_id: str,
    vehicle_id: str,
    source_node: str,
) -> List[AlertEvent]:
    events: List[AlertEvent] = []
    for d in detections:
        range_m = float(d.details.get('range_m', np.hypot(d.x, d.y)))
        if not params.event_min_range_m <= range_m <= params.event_max_range_m:
            continue
        in_lane = abs(d.y) < 3.0
        if d.radius_m >= 0.7 and in_lane and range_m < 20.0:
            severity = Severity.CRITICAL
        elif d.radius_m >= 0.5 or in_lane:
            severity = Severity.WARN
        else:
            severity = Severity.INFO
        events.append(
            AlertEvent(
                type='rock',
                severity=severity,
                confidence=d.confidence,
                label=_event_label('rock', f'{2.0 * d.radius_m:.1f} m across'),
                source_node=source_node,
                vehicle_id=vehicle_id,
                frame_id=frame_id,
                t_vehicle=t,
                x=d.x,
                y=d.y,
                z=d.z,
                geometry={'kind': 'sphere', 'radius_m': d.radius_m, 'extent_m': list(d.extent_m)},
                details={
                    'class': 'rock',
                    'radius_m': d.radius_m,
                    'range_m': range_m,
                    'lateral_m': d.y,
                    'in_lane': in_lane,
                    'point_count': d.point_count,
                    **d.details,
                },
            )
        )
    return events


def _bund_events(
    segments: List[BundSegment],
    params: SemanticParams,
    *,
    t: float,
    frame_id: str,
    vehicle_id: str,
    source_node: str,
) -> List[AlertEvent]:
    """Height-deficit and gap events from the classified shoulder runs."""
    events: List[AlertEvent] = []
    for seg in segments:
        mid_x = 0.5 * (seg.x_start + seg.x_end)
        if not params.event_min_range_m <= float(np.hypot(mid_x, seg.y)) <= params.event_max_range_m:
            continue
        if seg.state == 'low':
            deficit = params.bund_spec_height_m - seg.min_height_m
            severity = Severity.CRITICAL if seg.min_height_m < 0.5 * params.bund_min_height_m else Severity.WARN
            events.append(
                AlertEvent(
                    type='bund_low',
                    severity=severity,
                    confidence=float(np.clip(0.55 + 0.45 * deficit / max(params.bund_spec_height_m, 1e-3), 0.0, 1.0)),
                    label=_event_label(
                        'bund_low',
                        f'{seg.min_height_m:.2f} m of {params.bund_spec_height_m:.2f} m · {seg.side}',
                    ),
                    source_node=source_node,
                    vehicle_id=vehicle_id,
                    frame_id=frame_id,
                    t_vehicle=t,
                    x=mid_x,
                    y=seg.y,
                    z=seg.z,
                    geometry={'kind': 'segment', 'length_m': seg.length_m, 'side': seg.side},
                    details={
                        'class': 'bund_low',
                        'side': seg.side,
                        'min_height_m': seg.min_height_m,
                        'mean_height_m': seg.mean_height_m,
                        'spec_height_m': params.bund_spec_height_m,
                        'min_required_m': params.bund_min_height_m,
                        'deficit_m': float(deficit),
                        'length_m': seg.length_m,
                        'bins': seg.bins,
                    },
                )
            )
        elif seg.state == 'missing' and seg.length_m >= params.gap_length_m:
            events.append(
                AlertEvent(
                    type='bund_gap',
                    severity=Severity.CRITICAL if seg.length_m >= 2.0 * params.gap_length_m else Severity.WARN,
                    confidence=float(np.clip(seg.length_m / (3.0 * max(params.gap_length_m, 1e-3)), 0.4, 1.0)),
                    label=_event_label('bund_gap', f'{seg.length_m:.0f} m · {seg.side}'),
                    source_node=source_node,
                    vehicle_id=vehicle_id,
                    frame_id=frame_id,
                    t_vehicle=t,
                    x=mid_x,
                    y=seg.y,
                    z=seg.z,
                    geometry={'kind': 'segment', 'length_m': seg.length_m, 'side': seg.side},
                    details={
                        'class': 'bund_gap',
                        'side': seg.side,
                        'length_m': seg.length_m,
                        'gap_length_m': params.gap_length_m,
                        'bins': seg.bins,
                        'reason': 'no_crest',
                    },
                )
            )
    return events


def classify_frame(
    points: np.ndarray,
    *,
    params: Optional[SemanticParams] = None,
    t: float = 0.0,
    frame_id: str = 'base_link',
    vehicle_id: str = 'haul-01',
    source_node: str = 'edge_semantics',
) -> FrameSemantics:
    """Classify one body-frame scan and derive detections plus events."""
    p = params or SemanticParams()
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError('points must have shape (N, 3) or wider')
    points = np.ascontiguousarray(points[:, :3])
    n = int(points.shape[0])
    labels = np.full((n,), int(Label.UNKNOWN), dtype=np.int16)
    conf = np.zeros((n,), dtype=np.float32)
    if n == 0:
        return FrameSemantics(
            labels=labels,
            conf=conf,
            hag=np.zeros((0,), dtype=np.float32),
            road_half_width_m=float(p.road_half_width_m),
            counts=_counts(labels),
        )

    ground = estimate_ground_grid(
        points,
        cell_m=p.ground_cell_m,
        ground_band_m=p.ground_band_m,
        min_cell_points=max(2, int(p.min_bin_points)),
    )
    hag = ground.hag.astype(np.float64)
    road_hw = estimate_road_half_width(points, p)

    y = points[:, 1]
    known = np.isfinite(hag)
    ground_mask = known & (np.abs(hag) <= p.ground_band_m)
    road_mask = ground_mask & (np.abs(y) <= road_hw) & (np.abs(points[:, 0]) <= p.corridor_max_x_m)
    labels[road_mask] = int(Label.ROAD)
    labels[ground_mask & ~road_mask] = int(Label.GROUND)
    conf[ground_mask] = np.clip(
        1.0 - np.abs(hag[ground_mask]) / max(p.ground_band_m, 1e-3), 0.35, 1.0
    ).astype(np.float32)

    segments = _bund_segments(points, hag, labels, conf, road_hw, p)

    # Rocks compete only for non-ground points the bund pass did not claim.
    free = known & (hag > p.obstacle_min_hag_m) & (labels == int(Label.UNKNOWN))
    free_idx = np.flatnonzero(free)
    detections: List[Detection] = []
    if free_idx.size:
        cluster_idx = free_idx
        if cluster_idx.size > p.max_cluster_points:
            keep = np.linspace(0, cluster_idx.size - 1, p.max_cluster_points).astype(int)
            cluster_idx = cluster_idx[keep]
        detections = detect_rocks(points[cluster_idx], hag[cluster_idx], params=p.rock)
        for d in detections:
            d.frame_id = frame_id
        _rock_label_points(points, labels, conf, free_idx, detections)

    obstacle_idx = np.flatnonzero(free & (labels == int(Label.UNKNOWN)))
    if obstacle_idx.size:
        labels[obstacle_idx] = int(Label.OBSTACLE)
        conf[obstacle_idx] = np.clip(hag[obstacle_idx] / 1.5, 0.2, 1.0).astype(np.float32)

    events = _rock_events(
        detections, p, t=t, frame_id=frame_id, vehicle_id=vehicle_id, source_node=source_node
    )
    events += _bund_events(
        segments, p, t=t, frame_id=frame_id, vehicle_id=vehicle_id, source_node=source_node
    )

    for seg in segments:
        if seg.state == 'missing':
            continue
        mid_x = 0.5 * (seg.x_start + seg.x_end)
        detections.append(
            Detection(
                type='bund_low' if seg.state == 'low' else 'bund',
                confidence=0.8 if np.isfinite(seg.min_height_m) else 0.5,
                frame_id=frame_id,
                x=mid_x,
                y=seg.y,
                z=seg.z,
                extent_m=(seg.length_m, float(p.shoulder_band_m), float(seg.mean_height_m)),
                label=int(Label.BUND_LOW if seg.state == 'low' else Label.BUND),
                details={
                    'side': seg.side,
                    'min_height_m': seg.min_height_m,
                    'mean_height_m': seg.mean_height_m,
                    'length_m': seg.length_m,
                    'bins': seg.bins,
                },
            )
        )

    return FrameSemantics(
        labels=labels,
        conf=conf,
        hag=hag.astype(np.float32),
        road_half_width_m=float(road_hw),
        detections=detections,
        events=events,
        segments=segments,
        counts=_counts(labels),
    )


def _counts(labels: np.ndarray) -> Dict[str, int]:
    out: Dict[str, int] = {}
    if labels.size == 0:
        return out
    values, counts = np.unique(labels, return_counts=True)
    for value, count in zip(values.tolist(), counts.tolist()):
        out[class_style(int(value)).key] = int(count)
    return out
