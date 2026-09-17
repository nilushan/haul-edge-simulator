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
    # Fallback corridor width when the toe estimate has too little support.
    # Haul roads run three to four trucks abreast, so this is tens of metres.
    road_half_width_m: float = 12.0
    min_road_half_width_m: float = 3.0
    max_road_half_width_m: float = 24.0
    road_margin_m: float = 0.8
    shoulder_band_m: float = 4.0
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
    # A crest is only measurable when the scan reaches past it and comes back
    # down the far side. Looking along a distant shoulder the beams skim over
    # the crest and stop on the inner flank, which reads as a low bund.
    crest_observed_beyond_m: float = 0.4
    crest_drop_beyond_m: float = 0.25
    # Absence of returns only means a missing bund on a shoulder close enough
    # for the scan to resolve it at all.
    gap_max_edge_m: float = 13.0
    # An unsampled shoulder only counts as absent within this range.
    gap_observed_max_x_m: float = 22.0
    # A shoulder whose terrain keeps climbing past the berm line is a cut
    # batter, not a missing bund: nothing has to stop a truck falling uphill.
    cut_probe_min_m: float = 3.0
    cut_probe_max_m: float = 12.0
    cut_min_rise_m: float = 2.0
    obstacle_min_hag_m: float = 0.25
    # Ceiling on points handed to the clusterer, so a dense frame cannot
    # stall an edge node or the viz tick pump.
    max_cluster_points: int = 2000
    event_min_range_m: float = 6.0
    event_max_range_m: float = 30.0
    rock: RockParams = field(default_factory=RockParams)


@dataclass
class Corridor:
    """
    Distance from the vehicle to each edge of the running surface.

    A wide haul road curves away within the look-ahead, so the edge is held per
    longitudinal bin; ``left_m`` / ``right_m`` are the representative values
    used for reporting and as the fallback for bins with no measurement.
    """

    left_m: float
    right_m: float
    left_bins: Dict[int, float] = field(default_factory=dict)
    right_bins: Dict[int, float] = field(default_factory=dict)

    @property
    def half_width_m(self) -> float:
        return 0.5 * (self.left_m + self.right_m)

    @property
    def width_m(self) -> float:
        return self.left_m + self.right_m

    def edge(self, side: str) -> float:
        return self.left_m if side == 'left' else self.right_m

    def bin_edges(self, side: str) -> Dict[int, float]:
        return self.left_bins if side == 'left' else self.right_bins

    def edge_at(self, side: str, bin_index: int) -> float:
        return self.bin_edges(side).get(int(bin_index), self.edge(side))

    def per_point(self, side: str, bins: np.ndarray) -> np.ndarray:
        """Edge distance for every point, from its own longitudinal bin."""
        edges = np.full(bins.shape, self.edge(side), dtype=np.float64)
        for bin_index, value in self.bin_edges(side).items():
            edges[bins == bin_index] = value
        return edges

    def to_dict(self) -> Dict[str, float]:
        return {
            'left_m': self.left_m,
            'right_m': self.right_m,
            'width_m': self.width_m,
            'half_width_m': self.half_width_m,
        }


@dataclass
class SideProfile:
    """What one shoulder is: a falling edge that needs a bund, or a batter."""

    side: str
    kind: str  # 'fill' | 'cut'
    edge_m: float
    rise_m: float  # terrain height change just beyond the shoulder
    segments: List['BundSegment'] = field(default_factory=list)


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
    corridor: Corridor
    detections: List[Detection] = field(default_factory=list)
    events: List[AlertEvent] = field(default_factory=list)
    segments: List[BundSegment] = field(default_factory=list)
    sides: List[SideProfile] = field(default_factory=list)
    counts: Dict[str, int] = field(default_factory=dict)

    @property
    def road_half_width_m(self) -> float:
        return self.corridor.half_width_m


def _side_sign(side: str) -> float:
    return 1.0 if side == 'left' else -1.0


def fit_road_plane(
    points: np.ndarray,
    params: SemanticParams,
) -> Optional[Tuple[float, float, float]]:
    """
    Least-squares plane through the running surface: ``z = a·x + b·y + c``.

    A constant height per longitudinal bin is only valid if the road is level
    across its width in the body frame, and it is not: a couple of degrees of
    vehicle roll lifts one side of a 30 m road by most of a metre, which is
    more than a bund is tall. Seeding from the lane the truck is in and
    re-fitting over the points that agree with the plane recovers the surface
    including its roll, pitch, and cross-fall.
    """
    p = params
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    usable = np.all(np.isfinite(points), axis=1) & (np.abs(x) <= p.corridor_max_x_m)
    selected = usable & (np.abs(y) <= 3.0)
    if int(np.count_nonzero(selected)) < 20:
        return None

    plane: Optional[Tuple[float, float, float]] = None
    for _ in range(3):
        idx = np.flatnonzero(selected)
        design = np.column_stack([x[idx], y[idx], np.ones(idx.size)])
        try:
            coefficients, *_ = np.linalg.lstsq(design, z[idx], rcond=None)
        except np.linalg.LinAlgError:
            return plane
        plane = (float(coefficients[0]), float(coefficients[1]), float(coefficients[2]))
        residual = z - (plane[0] * x + plane[1] * y + plane[2])
        grown = usable & (np.abs(residual) <= 0.25) & (np.abs(y) <= p.max_road_half_width_m)
        if int(np.count_nonzero(grown)) < 30:
            break
        selected = grown
    return plane


def _reference_z(
    points: np.ndarray,
    params: SemanticParams,
) -> Tuple[np.ndarray, np.ndarray, Dict[int, bool], float]:
    """
    Per-point road height, longitudinal bin, and which bins saw the road.

    Falls back to a flat reference from the lane band when the cloud is too
    sparse to fit a plane.
    """
    p = params
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    usable = np.all(np.isfinite(points), axis=1) & (np.abs(x) <= p.corridor_max_x_m)
    bins = np.full(points.shape[0], -1, dtype=np.int64)
    reference = np.full(points.shape[0], np.nan, dtype=np.float64)
    if not np.any(usable):
        return bins, reference, {}, 0.0

    x_min = float(np.min(x[usable]))
    bins[usable] = np.floor((x[usable] - x_min) / p.x_bin_m).astype(np.int64)

    plane = fit_road_plane(points, p)
    if plane is not None:
        reference[usable] = plane[0] * x[usable] + plane[1] * y[usable] + plane[2]
    else:
        lane = usable & (np.abs(y) <= 2.5)
        for b in np.unique(bins[lane]):
            sel = np.flatnonzero(lane & (bins == b))
            if sel.size >= p.min_bin_points:
                reference[bins == b] = float(np.percentile(z[sel], 20.0))

    lane = usable & (np.abs(y) <= 3.0)
    observed: Dict[int, bool] = {}
    for b in np.unique(bins[lane]):
        sel = np.flatnonzero(lane & (bins == b))
        if sel.size >= p.min_bin_points:
            observed[int(b)] = True
    return bins, reference, observed, x_min


def _smooth_bin_series(values: Dict[int, float], window: int = 5) -> Dict[int, float]:
    """
    Rolling median over neighbouring bins.

    A rock sitting near the shoulder corrupts the toe in its own bin; the
    shoulder itself is continuous, so the neighbours outvote it.
    """
    if not values:
        return {}
    keys = sorted(values)
    half = max(int(window) // 2, 1)
    smoothed: Dict[int, float] = {}
    for i, key in enumerate(keys):
        window_keys = keys[max(0, i - half): i + half + 1]
        smoothed[key] = float(np.median([values[k] for k in window_keys]))
    return smoothed


def estimate_corridor(points: np.ndarray, params: SemanticParams) -> Corridor:
    """
    Distance to each edge of the running surface, per side and per bin.

    A truck runs in a lane, so the near shoulder can be a few metres away while
    the far one is twenty; a single symmetric width would put the shoulder band
    in the wrong place on both sides. Within each longitudinal bin the
    innermost clearly raised return marks that side's edge, which also follows
    the road as it curves out of the body-frame band.
    """
    p = params
    fallback = float(np.clip(p.road_half_width_m, p.min_road_half_width_m, p.max_road_half_width_m))
    bins, reference, observed, _ = _reference_z(points, p)
    if not observed:
        return Corridor(fallback, fallback)

    y, z = points[:, 1], points[:, 2]
    above = np.where(np.isfinite(reference), z - reference, np.nan)
    representative: Dict[str, float] = {}
    per_bin: Dict[str, Dict[int, float]] = {}
    for side in ('left', 'right'):
        sign = _side_sign(side)
        lateral = sign * y
        candidates = (bins >= 0) & (lateral > 2.5) & (lateral <= p.max_road_half_width_m + 6.0)
        toes: Dict[int, float] = {}
        for b in observed:
            sel = np.flatnonzero(candidates & (bins == b))
            if sel.size == 0:
                continue
            raised = sel[above[sel] >= p.bund_present_min_m]
            if raised.size:
                toes[int(b)] = float(
                    np.clip(
                        np.min(lateral[raised]),
                        p.min_road_half_width_m,
                        p.max_road_half_width_m,
                    )
                )
        smoothed = _smooth_bin_series(toes)
        per_bin[side] = smoothed
        representative[side] = (
            float(np.median(list(smoothed.values()))) if len(smoothed) >= 3 else fallback
        )
    return Corridor(
        left_m=representative['left'],
        right_m=representative['right'],
        left_bins=per_bin['left'],
        right_bins=per_bin['right'],
    )


def estimate_road_half_width(points: np.ndarray, params: SemanticParams) -> float:
    """Mean of the two corridor edges (see :func:`estimate_corridor`)."""
    return estimate_corridor(points, params).half_width_m


def _side_profile(
    points: np.ndarray,
    labels: np.ndarray,
    conf: np.ndarray,
    side: str,
    corridor: Corridor,
    bins: np.ndarray,
    reference: np.ndarray,
    observed: Dict[int, bool],
    x_min: float,
    params: SemanticParams,
) -> SideProfile:
    """
    Classify one shoulder: cut batter, or a falling edge that needs a bund.

    Crest height is measured against the road surface in the same longitudinal
    bin, not against the local ground grid: inside a berm the grid estimate
    rides up with the berm itself and would report a few centimetres.
    """
    p = params
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    sign = _side_sign(side)
    lateral = sign * y
    usable = (bins >= 0) & np.isfinite(reference)
    edge_m = corridor.edge(side)
    edges = corridor.per_point(side, bins)
    above = np.where(np.isfinite(reference), z - reference, np.nan)

    # Does the ground keep climbing past where a berm would sit? Then this is
    # the high side of the road and no bund is expected.
    rises: List[float] = []
    probe = usable & (lateral >= edges + p.cut_probe_min_m) & (lateral <= edges + p.cut_probe_max_m)
    for b in observed:
        sel = np.flatnonzero(probe & (bins == b))
        if sel.size >= p.min_bin_points:
            rises.append(float(np.percentile(above[sel], 75.0)))
    rise = float(np.median(rises)) if len(rises) >= 2 else 0.0
    kind = 'cut' if rise >= p.cut_min_rise_m else 'fill'

    inner = edges - p.road_margin_m
    outer = edges + p.shoulder_band_m
    if kind == 'cut':
        # The whole batter is one class: it is terrain, not an obstruction.
        batter = np.flatnonzero(usable & (lateral >= inner) & (above > 0.3))
        if batter.size:
            labels[batter] = int(Label.CUT_SLOPE)
            conf[batter] = float(np.clip(rise / max(p.cut_min_rise_m * 2.0, 1e-3), 0.4, 1.0))
        return SideProfile(side=side, kind=kind, edge_m=edge_m, rise_m=rise)

    shoulder_idx = np.flatnonzero(usable & (lateral >= inner) & (lateral <= outer))
    states: Dict[int, Tuple[str, float, float, float]] = {}
    for b in sorted(observed):
        bin_edge = corridor.edge_at(side, b)
        sel = shoulder_idx[bins[shoulder_idx] == b] if shoulder_idx.size else np.zeros((0,), dtype=np.int64)
        if sel.size < p.min_bin_points:
            bin_x = x_min + (b + 0.5) * p.x_bin_m
            in_range = (
                abs(float(bin_x)) <= p.gap_observed_max_x_m
                and bin_edge <= p.gap_max_edge_m
            )
            states[b] = (
                'missing' if in_range else 'unknown',
                float('nan'),
                sign * (bin_edge + 1.0),
                0.0,
            )
            continue

        crest = sel[int(np.argmax(above[sel]))]
        height = float(above[crest])
        raised = sel[above[sel] > 0.5 * p.bund_present_min_m]
        # The top was seen only if the scan carried on past the crest and came
        # back down; otherwise the beam simply stopped on the inner flank.
        beyond = sel[lateral[sel] > lateral[crest] + p.crest_observed_beyond_m]
        measurable = bool(
            beyond.size and float(np.min(above[beyond])) <= height - p.crest_drop_beyond_m
        )
        close_enough = bin_edge <= p.gap_max_edge_m

        if height < p.bund_present_min_m:
            absent = close_enough and (
                measurable or abs(float(x[crest])) <= p.gap_observed_max_x_m
            )
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

    return SideProfile(
        side=side,
        kind=kind,
        edge_m=edge_m,
        rise_m=rise,
        segments=_group_runs(side, states, x_min, p),
    )


def _group_runs(
    side: str,
    states: Dict[int, Tuple[str, float, float, float]],
    x_min: float,
    params: SemanticParams,
) -> List[BundSegment]:
    """Collapse per-bin states into contiguous runs of the same finding."""
    ordered = sorted(states)
    segments: List[BundSegment] = []
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
                x_start=x_min + run_start * params.x_bin_m,
                x_end=x_min + (b + 1) * params.x_bin_m,
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
            corridor=Corridor(p.road_half_width_m, p.road_half_width_m),
            counts=_counts(labels),
        )

    ground = estimate_ground_grid(
        points,
        cell_m=p.ground_cell_m,
        ground_band_m=p.ground_band_m,
        min_cell_points=max(2, int(p.min_bin_points)),
    )
    hag = ground.hag.astype(np.float64)
    corridor = estimate_corridor(points, p)
    bins, reference, observed, x_min = _reference_z(points, p)

    y = points[:, 1]
    known = np.isfinite(hag)
    ground_mask = known & (np.abs(hag) <= p.ground_band_m)
    left_edge = corridor.per_point('left', bins)
    right_edge = corridor.per_point('right', bins)
    inside_corridor = (y <= left_edge) & (y >= -right_edge)
    road_mask = ground_mask & inside_corridor & (np.abs(points[:, 0]) <= p.corridor_max_x_m)
    labels[road_mask] = int(Label.ROAD)
    labels[ground_mask & ~road_mask] = int(Label.GROUND)
    conf[ground_mask] = np.clip(
        1.0 - np.abs(hag[ground_mask]) / max(p.ground_band_m, 1e-3), 0.35, 1.0
    ).astype(np.float32)

    sides = [
        _side_profile(
            points,
            labels,
            conf,
            side,
            corridor,
            bins,
            reference,
            observed,
            x_min,
            p,
        )
        for side in ('left', 'right')
    ]
    # Only a falling shoulder can be missing a bund.
    segments = [seg for profile in sides if profile.kind == 'fill' for seg in profile.segments]

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
        corridor=corridor,
        detections=detections,
        events=events,
        segments=segments,
        sides=sides,
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
