# Code structure

Libs and ROS nodes are **separated by folder**. Colcon still discovers every
package under `ws/src/` recursively.

```text
ws/src/
├── libs/                         # pure libraries (no ROS nodes)
│   ├── edge_sim/                 # maps, sensor models, StreamHub, topic contract
│   │   └── edge_sim/
│   │       ├── maps.py
│   │       ├── models/           # world (cut/fill terrain), vehicle, imu, gnss, lidar
│   │       ├── stream/           # StreamHub + JSONL format
│   │       ├── topics.py         # shared bus topic names
│   │       └── tools/
│   └── edge_perception/          # detection algorithms + schemas
│       └── edge_perception/
│           ├── schema.py         # Detection / AlertEvent + class & event styles
│           ├── semantics.py      # whole-frame classifier (classes + events)
│           ├── geometry.py
│           ├── ground.py
│           ├── rocks.py
│           ├── bunds.py
│           ├── tracking.py       # pose transform + spatial event dedup
│           ├── vibration.py
│           └── cloud_io.py
│
├── nodes/                        # ROS node packages (thin wrappers)
│   ├── edge_sensor_source/       # StreamHub → bus topics
│   ├── edge_processor/           # example /lidar → /edge/lidar/processed
│   ├── edge_rock_detect/         # rocks cloud + alerts
│   ├── edge_bund_detect/         # bunds cloud + alerts
│   ├── edge_vibe_detect/         # IMU vibration alerts
│   ├── edge_event_store/         # /edge/alerts → SQLite
│   └── edge_viz/                 # browser UI (hub or bus mode)
│
└── bringup/                      # launch + config only
    └── edge_bringup/
        ├── launch/
        └── config/
```

## Why split libs vs nodes?

| Folder | Contains | Depends on ROS? |
|---|---|---|
| `libs/` | algorithms, models, schemas, stream I/O | No (except optional message helpers) |
| `nodes/` | `rclpy` nodes, topic wiring, params | Yes |
| `bringup/` | launch files + YAML | launch only |

Rules:

- Detectors **call** `edge_perception` — they do not reimplement geometry.
- Sources **call** `edge_sim` — they do not own world math.
- Viz never imports detector internals; it only subscribes to bus topics.
- You can unit-test libs with plain `pytest` (no ROS daemon).

## Data flow

```text
DEV
  edge_sim.StreamHub ──► PerceptionOverlay (same classifier) ──► edge_viz ──► browser

EDGE
  edge_sensor_source ──publish──► ROS bus (/imu /gnss /lidar /odom)
        │
        ├─► edge_rock_detect  ──► /edge/lidar/semantic /edge/lidar/rocks /edge/alerts
        ├─► edge_bund_detect  ──► /edge/lidar/bunds  /edge/detections /edge/alerts
        ├─► edge_vibe_detect  ──► /edge/vibe/features /edge/alerts
        ├─► edge_event_store  ──► SQLite (local edge storage)
        └─► edge_viz.run_from_bus ──► browser
```

## Editable PYTHONPATH

```bash
source scripts/env_pythonpath.sh
python3 -m edge_viz.run_from_hub
pytest ws/src/libs/edge_perception/test
```

## Commands

```bash
# Dev UI (no ROS)
./scripts/run_viz.sh

# Edge stack (all detector nodes)
ros2 launch edge_bringup edge_vehicle.launch.py

# Source only
ros2 launch edge_bringup sensor_source.launch.py

# Colcon (from ws/)
colcon build --symlink-install
```
