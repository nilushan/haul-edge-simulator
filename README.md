# Haul edge sensor sim

Synthetic on-vehicle sensors for haul-truck edge development.

## Layout (`ws/src`)

| Folder | What |
|---|---|
| **`libs/`** | Pure libraries — no ROS nodes |
| **`nodes/`** | ROS node packages |
| **`bringup/`** | Launch + config |

| Package | Folder | Role |
|---|---|---|
| **edge_sim** | libs | Maps + sensor models + StreamHub |
| **edge_perception** | libs | Frame classification, events, class/event styles |
| **edge_sensor_source** | nodes | Sim → `/imu` `/gnss` `/lidar` `/odom` |
| **edge_rock_detect** | nodes | `/lidar` → `/edge/lidar/rocks` + alerts |
| **edge_bund_detect** | nodes | `/lidar` → `/edge/lidar/bunds` + alerts |
| **edge_vibe_detect** | nodes | `/imu` → vibration alerts |
| **edge_event_store** | nodes | `/edge/alerts` → SQLite |
| **edge_processor** | nodes | Example `/lidar` → `/edge/*` stub |
| **edge_viz** | nodes | Browser UI (hub or bus) |
| **edge_bringup** | bringup | Launch + config |

See [`docs/STRUCTURE.md`](docs/STRUCTURE.md).

## Quick start

```bash
# Dev UI (StreamHub → browser, no ROS)
./scripts/run_viz.sh
# open http://127.0.0.1:8099/

# Docker
./start.sh              # same dev UI
./start.sh --edge       # full edge stack (detectors + viz)
./start.sh --ros        # sensor source node only
```

## Edge stack

```bash
ros2 launch edge_bringup edge_vehicle.launch.py
# nodes:
#   edge_sensor_source / sensor_source
#   edge_rock_detect   / edge_rock_detect
#   edge_bund_detect   / edge_bund_detect
#   edge_vibe_detect   / edge_vibe_detect
#   edge_event_store   / edge_event_store
#   edge_viz           / viz_from_bus
```

Bag-only (no sim source):

```bash
ros2 launch edge_bringup edge_vehicle.launch.py use_sim:=false
ros2 bag play your_drive.mcap   # remap onto bus topics
```

## Record streams

```bash
./scripts/generate_streams.sh
STREAM_MODE=replay ./scripts/run_viz.sh
```

## Detection classes and events

Every LiDAR frame is classified into **road**, **ground**, **bund**,
**bund low**, **rock**, **non-ground** and **unclassified**, and the classes
drive both the 3D colours and the toggles in the browser. Findings worth acting
on are raised as events and frozen on the map where they were first seen, each
with a colour-coded marker and a text tag:

| Event | Map tag |
|---|---|
| Rock in or beside the lane | `ROCK · 1.2 m across` |
| Berm below the compliance height | `BUND LOW · 0.74 m of 1.65 m · left` |
| No berm over a stretch of shoulder | `BUND GAP · 18 m · right` |
| Ride roughness over threshold | `VIBE` |

The class and event styles come from one table
(`edge_perception.schema.style_catalog`, served at `/api/classes`), so the
legend, toggles, point colours, markers and event feed stay in step.

Every map seeds a few under-built and missing berm sections
(`MapSpec.n_bund_defects`) so there is something real for the bund checks to
find. See [`docs/EDGE_ARCHITECTURE.md`](docs/EDGE_ARCHITECTURE.md) for the
thresholds and what the detector deliberately refuses to claim.

## Maps

`haul_corridor` · `tight_switchbacks` · `open_pit_bench` · `rocky_descent`

## Architecture note

- **Libs** (`ws/src/libs`) hold algorithms and schemas; unit-test without ROS.
- **Nodes** (`ws/src/nodes`) are thin ROS wrappers around libs.
- **Dev:** browser reads StreamHub in-process (`edge_viz.run_from_hub`), with
  the same perception pass running inline (`--no-inline-detect` turns it off).
- **Edge:** browser UI is fed by ROS subscriptions (`edge_viz.run_from_bus`).  
- Browser never speaks ROS; it only uses WebSocket `/ws`.
