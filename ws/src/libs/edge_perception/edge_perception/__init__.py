"""Public API for the ROS-independent perception library."""

from edge_perception.bunds import (
    BundCrest,
    BundParams,
    bund_points_from_detections,
    detect_bunds,
    extract_crest,
)
from edge_perception.cloud_io import decode_xyz, encode_xyz_label
from edge_perception.geometry import euclidean_clusters, pca_extents, roi_mask, voxel_downsample
from edge_perception.ground import GroundResult, estimate_ground_grid
from edge_perception.rocks import RockParams, detect_rocks
from edge_perception.schema import (
    AlertEvent,
    ClassStyle,
    Detection,
    DetectionSet,
    EventStyle,
    Label,
    Severity,
    class_style,
    event_style,
    json_safe,
    style_catalog,
)
from edge_perception.semantics import (
    BundSegment,
    FrameSemantics,
    SemanticParams,
    classify_frame,
    estimate_road_half_width,
)
from edge_perception.tracking import Pose2D, SpatialDeduplicator, body_to_map
from edge_perception.vibration import ImuSample, VibeFeatures, VibeParams, VibrationMonitor

__all__ = [
    'AlertEvent',
    'BundCrest',
    'BundParams',
    'BundSegment',
    'ClassStyle',
    'Detection',
    'DetectionSet',
    'EventStyle',
    'FrameSemantics',
    'GroundResult',
    'ImuSample',
    'Label',
    'Pose2D',
    'RockParams',
    'SemanticParams',
    'Severity',
    'SpatialDeduplicator',
    'VibeFeatures',
    'VibeParams',
    'VibrationMonitor',
    'body_to_map',
    'bund_points_from_detections',
    'class_style',
    'classify_frame',
    'decode_xyz',
    'detect_bunds',
    'detect_rocks',
    'encode_xyz_label',
    'estimate_ground_grid',
    'estimate_road_half_width',
    'euclidean_clusters',
    'event_style',
    'extract_crest',
    'json_safe',
    'pca_extents',
    'roi_mask',
    'style_catalog',
    'voxel_downsample',
]
