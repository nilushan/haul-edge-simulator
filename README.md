# Haul edge sensor sim

Synthetic on-vehicle sensors for haul-truck edge development.

## Packages (`ws/src`)

| Package | What it is |
|---|---|
| **edge_sim** | Maps + sensor models + StreamHub (no ROS nodes) |
| **edge_sensor_source** | ROS node: sim → `/imu/data` `/gnss/fix` `/lidar` `/odom` |
| **edge_processor** | ROS node: example `/lidar` → `/edge/*` |
| **edge_viz** | Browser UI (from hub or from ROS bus) |
| **edge_bringup** | Launch + config |

See [`docs/STRUCTURE.md`](docs/STRUCTURE.md).

## Quick start

```bash
# Dev UI (StreamHub → browser, no ROS)
./scripts/run_viz.sh
# open http://127.0.0.1:8099/

# Docker
./start.sh              # same dev UI
./start.sh --edge       # full edge stack (separate nodes)
./start.sh --ros        # sensor source node only
```

## Edge stack

```bash
ros2 launch edge_bringup edge_vehicle.launch.py
# nodes:
#   edge_sensor_source / sensor_source
#   edge_processor     / edge_processor
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

## Maps

`haul_corridor` · `tight_switchbacks` · `open_pit_bench` · `rocky_descent`

## Architecture note

- **Dev:** browser reads StreamHub in-process (`edge_viz.run_from_hub`).  
- **Edge:** browser UI is fed by ROS subscriptions (`edge_viz.run_from_bus` → `bus_ingress`).  
- Browser never speaks ROS; it only uses WebSocket `/ws`.
