# Vehicle edge architecture

## Layout

| Folder | Role |
|---|---|
| `ws/src/libs/` | Libraries only (`edge_sim`, `edge_perception`) |
| `ws/src/nodes/` | ROS node packages (source, detectors, viz, store) |
| `ws/src/bringup/` | Launch + shared config |

## Packages

| Package | Kind | Role |
|---|---|---|
| `edge_sim` | lib | Fake sensors + multi-map StreamHub + topic contract |
| `edge_perception` | lib | Ground / rocks / bunds / vibration + alert schemas |
| `edge_sensor_source` | node | Publish sim/replay onto bus topics |
| `edge_processor` | node | Example processing on the bus (optional stub) |
| `edge_rock_detect` | node | Rock clouds + detections + alerts |
| `edge_bund_detect` | node | Bund clouds + detections + alerts |
| `edge_vibe_detect` | node | Excessive vibration alerts from IMU |
| `edge_event_store` | node | Persist alerts to SQLite |
| `edge_viz` | node | Browser UI (hub mode or bus mode) |
| `edge_bringup` | bringup | Launch files + shared config |

## Bus topics (`edge_sim.topics`)

| Topic | Type | Direction |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | source → |
| `/gnss/fix` | `sensor_msgs/NavSatFix` | source → |
| `/lidar` | `sensor_msgs/PointCloud2` | source → |
| `/odom` | `nav_msgs/Odometry` | source → |
| `/edge/lidar/processed` | `PointCloud2` | processor → |
| `/edge/lidar/ground` | `PointCloud2` | rock detect → |
| `/edge/lidar/obstacles` | `PointCloud2` | rock detect → |
| `/edge/lidar/rocks` | `PointCloud2` | rock detect → |
| `/edge/lidar/bunds` | `PointCloud2` | bund detect → |
| `/edge/detections` | `std_msgs/String` JSON | detectors → |
| `/edge/alerts` | `std_msgs/String` JSON | detectors → |
| `/edge/vibe/features` | `std_msgs/String` JSON | vibe → |
| `/edge/status` | `std_msgs/String` JSON | all nodes → |

Detection clouds use fields: `x y z intensity label conf`.

## Gradual realism

1. **Now:** `edge_sensor_source` (sim) on the bus  
2. **Next:** `ros2 bag play` with remaps, `use_sim:=false`  
3. **Then:** real driver packages publishing the same names  
4. **Processors:** libs stay; only node wiring changes  

Viz in bus mode (`edge_viz.run_from_bus`) never cares which source is upstream.

## Diagram

```text
 edge_sim (lib)          edge_sensor_source (node)
 models/stream    ──►    publish bus topics
                                │
        ┌───────────────┬───────┼───────────┬──────────────┐
        ▼               ▼       ▼           ▼              ▼
 edge_rock_detect  edge_bund  edge_vibe  edge_processor  edge_viz
        │               │       │
        └───────────────┴───► /edge/alerts ──► edge_event_store
                                      │
                                      ▼
                                 SQLite (edge)
```
