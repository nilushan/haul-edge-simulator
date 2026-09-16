"""
Canonical vehicle-edge sensor bus contract.

All sources (sim, stream replay, rosbag, real drivers) should publish these
topics (or be remapped onto them). All processors and the bus-mode visualizer
subscribe here — they must not care which source is upstream.

  Sources ──publish──►  Sensor Bus (ROS 2)  ──subscribe──►  Processors / Viz
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, fields
from typing import Any, Dict


@dataclass(frozen=True)
class SensorBusContract:
    """Stable names + frames for the edge sensor bus."""

    # --- raw vehicle sensors (inputs) ---------------------------------
    imu_topic: str = '/imu/data'
    gnss_topic: str = '/gnss/fix'
    lidar_topic: str = '/lidar'
    # Ground-truth / fused pose when available (sim or localization)
    odom_topic: str = '/odom'
    # Legacy sim alias still accepted by ingress (see also_sim_odom)
    sim_odom_topic: str = '/sim/ground_truth/odom'

    # --- processed outputs (processors publish, viz may also show) ----
    processed_lidar_topic: str = '/edge/lidar/processed'
    edge_status_topic: str = '/edge/status'

    # --- frames -------------------------------------------------------
    frame_map: str = 'map'
    frame_base: str = 'base_link'
    frame_imu: str = 'imu_link'
    frame_lidar: str = 'lidar_link'

    # --- metadata -----------------------------------------------------
    bus_version: str = '1'

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any] | None) -> 'SensorBusContract':
        if not data:
            return cls()
        known = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in data.items() if k in known})

    def ros_parameters(self) -> Dict[str, Any]:
        """Flat dict suitable for ROS node parameters."""
        return self.to_dict()


def default_bus_contract() -> SensorBusContract:
    return SensorBusContract()


# Documented QoS intent (applied in nodes; not serialized on the wire as one profile).
#
# Raw sensors:  sensor_data / BEST_EFFORT, KEEP_LAST small depth
# Odom/status:  reliable, KEEP_LAST 10
BUS_QOS_NOTES = {
    'imu': 'sensor_data (best effort)',
    'gnss': 'sensor_data (best effort)',
    'lidar': 'sensor_data (best effort)',
    'odom': 'reliable depth 10',
    'edge_status': 'reliable depth 10',
}
