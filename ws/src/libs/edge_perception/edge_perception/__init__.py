"""Public API for the ROS-independent perception library."""

from edge_perception.bunds import (
    BundCrest,
    BundParams,
    bund_points_from_detections,
    detect_bunds,
    extract_crest,
)
from edge_perception.cloud_io import decode_xyz, encode_xyz_label
from edge_perception.geometry import pca_extents, roi_mask, voxel_downsample
from edge_perception.ground import GroundResult, estimate_ground_grid
from edge_perception.rocks import RockParams, detect_rocks
from edge_perception.schema import AlertEvent, Detection, DetectionSet, Label, Severity
from edge_perception.vibration import ImuSample, VibeFeatures, VibeParams, VibrationMonitor

__all__ = [
    'AlertEvent',
    'BundCrest',
    'BundParams',
    'Detection',
    'DetectionSet',
    'GroundResult',
    'ImuSample',
    'Label',
    'RockParams',
    'Severity',
    'VibeFeatures',
    'VibeParams',
    'VibrationMonitor',
    'bund_points_from_detections',
    'decode_xyz',
    'detect_bunds',
    'detect_rocks',
    'encode_xyz_label',
    'estimate_ground_grid',
    'extract_crest',
    'pca_extents',
    'roi_mask',
    'voxel_downsample',
]
