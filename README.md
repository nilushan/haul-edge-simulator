# Haul edge sensor sim

Synthetic **haul-truck** on-vehicle sensor suite for local development.

**Architecture:** build toward a real vehicle **edge node**.

- **Sensor bus (ROS topics)** is the stable contract — sim, bags, and real drivers all publish here.
- **Processors** and **viz (bus mode)** only subscribe to that bus.
- **StreamHub** is the sim/replay *source* used in dev; not what production consumers talk to.

Details: [`docs/EDGE_ARCHITECTURE.md`](docs/EDGE_ARCHITECTURE.md).

| Stream | ROS 2 type | Topic (default) |
|---|---|---|
| Vehicle motion (shared truth) | — | drives all sensors |
| **IMU** | `sensor_msgs/Imu` | `/imu/data` |
| **GNSS** | `sensor_msgs/NavSatFix` | `/gnss/fix` |
| **LiDAR** | `sensor_msgs/PointCloud2` | `/lidar` |
| Ground-truth pose (debug) | `nav_msgs/Odometry` | `/sim/ground_truth/odom` |

```text
haul-edge-sim/
├── data/streams/           # replayable multi-map streams (JSONL)
├── docker/                 # Jazzy runner
├── scripts/
│   ├── run_viz.sh          # host StreamHub + viz subscriber
│   ├── generate_streams.sh # pre-record all maps
│   └── generate_offline.py # legacy CSV dump
└── ws/src/edge_sensor_sim/
    ├── edge_sensor_sim/
    │   ├── maps.py         # map presets
    │   ├── stream/         # hub + replay format
    │   ├── stream_server.py
    │   ├── viz_server.py   # HTTP/WS subscriber UI
    │   ├── sensor_suite_node.py  # ROS subscriber/bridge
    │   └── models/         # pure Python IMU/GNSS/LiDAR/vehicle/world
    ├── launch/
    └── config/
```

## Maps

| id | Description |
|---|---|
| `haul_corridor` | Default snaking corridor, berms, mixed rocks |
| `tight_switchbacks` | Narrow road, sharp wiggles, tall bunds |
| `open_pit_bench` | Wide bench, long curve, big elevation steps |
| `rocky_descent` | Steep downhill, dense rock field |

Live mode **cycles** the playlist every `DURATION` seconds (configurable).

## Quick start (Docker)

```bash
./start.sh
# → sole StreamHub (live multi-map) + visualizer subscriber
# open http://127.0.0.1:8099/
```

| Command | What it runs |
|---|---|
| `./start.sh` | **Dev:** StreamHub → viz (direct) |
| `./start.sh --edge` | **Realistic:** sim → bus → processor → viz_bus |
| `./start.sh --all` | Dev shortcut: hub → ROS + direct viz |
| `./start.sh --ros` | Hub → ROS bus publishers only |
| `./start.sh --replay` | Replay `data/streams` (dev path) |
| `./start.sh --maps haul_corridor,rocky_descent` | Live map subset |
| `./start.sh --shell` | Interactive Jazzy shell |
| `./start.sh --build-only` | Build image and exit |

```bash
VIZ_PORT=8099 DURATION=60 MAPS=haul_corridor,open_pit_bench ./start.sh --all
```

## Host-only visualizer (no Docker)

```bash
pip3 install -r requirements.txt
./scripts/run_viz.sh
# open http://127.0.0.1:8099/
```

### Record replayable streams

```bash
./scripts/generate_streams.sh
# writes data/streams/<map_id>/{manifest.json,*.jsonl}

STREAM_MODE=replay ./scripts/run_viz.sh
```

## Data flow

### Edge-realistic (grow toward this)

```text
  sim | bag | real drivers
            │
            ▼
     ROS sensor bus     /imu/data /gnss/fix /lidar /odom
            │
     ┌──────┼──────────────┐
     ▼      ▼              ▼
 processor  viz_bus     other nodes
     │      │
     │      └── WS ──► browser
     └── /edge/lidar/processed , /edge/status
```

### Dev shortcut

```text
StreamHub ──WS──► browser     (no ROS required)
```

## Web visualizer API

| API | Description |
|---|---|
| `GET /` | Dashboard UI |
| `GET /api/healthz` | Liveness |
| `GET /api/status` | Latest samples + history tails |
| `GET /api/catalog` | Maps + recorded streams + active source |
| `POST /api/select` | `{"map_id":"..."}` or `{"stream_id":"..."}` |
| `GET /api/history/{imu\|gnss\|odom}` | Ring-buffer history |
| `GET /api/lidar` | Latest downsampled cloud |
| `GET /api/config` | Rates / mode / active map |
| `WS /ws` | Live ticks (~10 Hz) |

## Stream format

```text
data/streams/<id>/
  manifest.json   # map_id, rates, duration
  odom.jsonl
  imu.jsonl
  gnss.jsonl
  lidar.jsonl     # body-frame xy/z/intensity (viz + ROS replay)
```

## Manual ROS

```bash
./docker/build.sh && ./docker/shell.sh
# inside:
source /opt/ros/jazzy/setup.bash
cd /project/ws && colcon build --packages-select edge_sensor_sim && source install/setup.bash
# preferred: sole hub + ROS
python3 -m edge_sensor_sim.stream_server --ros --no-viz --maps haul_corridor
ros2 topic hz /lidar /imu/data /gnss/fix
```

## What is simulated (v1 models)

- **Vehicle:** map centerline, grade-aware speed, turns, bumps
- **IMU:** specific force + rates + bias/noise
- **GNSS:** ENU→lat/lon + noise/dropouts
- **LiDAR:** multi-ring ray cast into procedural haul world (not bag replay of random clouds)

## Frames

- `map` / ENU tangent at GNSS origin
- `base_link` — vehicle body
- `lidar_link`, `imu_link` — fixed offsets (TF static in launch)
