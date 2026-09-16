# Vehicle edge architecture (gradual realism)

Goal: **sim, recorded playback, and real sensors all look the same** to
processors and the visualizer.

## Canonical bus (the contract)

Everything important happens on a fixed set of ROS 2 topics:

| Topic | Type | Role |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | raw IMU |
| `/gnss/fix` | `sensor_msgs/NavSatFix` | raw GNSS |
| `/lidar` | `sensor_msgs/PointCloud2` | raw LiDAR |
| `/odom` | `nav_msgs/Odometry` | pose (sim GT or localization) |
| `/edge/lidar/processed` | `PointCloud2` | processor output |
| `/edge/status` | `std_msgs/String` (JSON) | processor heartbeat / diagnostics |

Defined in code: `edge_sensor_sim/bus/contract.py`  
Params: `config/bus.yaml`, `config/default.yaml`

**Rule:** sources *publish* the bus. Processors and viz *only subscribe*.
They must not import sim models or know whether data is fake or real.

```text
 ┌──────────────┐   ┌──────────────┐   ┌──────────────┐
 │ Sim StreamHub│   │ rosbag /     │   │ Real drivers │
 │ → ROS pub    │   │ MCAP play    │   │ (HW)         │
 └──────┬───────┘   └──────┬───────┘   └──────┬───────┘
        │                  │                  │
        └────────────┬─────┴──────────────────┘
                     ▼
              Sensor bus topics
                     │
        ┌────────────┼────────────┐
        ▼            ▼            ▼
  processor(s)   viz_bus      recorders /
  (subscribe)   (subscribe)   downstream
        │
        ▼
  /edge/lidar/processed
  /edge/status
```

## Two run modes

### 1) Dev shortcut (fast UI, no bus required)

```bash
./start.sh
# or host: ./scripts/run_viz.sh
```

```text
StreamHub ──WS/REST──► browser
```

Good for UI and map work. **Not** the production data path.

### 2) Edge-realistic path (what you grow toward)

```bash
./start.sh --edge
# inside container equivalent:
#   ros2 launch edge_sensor_sim edge_vehicle.launch.py
```

```text
sensor_suite_node (sim source)
        │ publishes bus topics
        ▼
processor_stub_node
        │ publishes /edge/*
        ▼
viz_bus_server  ──WS──► browser
   (subscribes to bus only)
```

## Gradual roadmap

| Stage | Source on the bus | Processors | Viz |
|---|---|---|---|
| **A** now | `sensor_suite_node` (sim) | `processor_stub` | `viz_bus_server` |
| **B** | `ros2 bag play` / stream→ROS | stub → real seg/detect | same viz_bus |
| **C** | real IMU/GNSS/LiDAR drivers | production nodes | same viz_bus |
| **D** | mixed (sim some, real others) | unchanged | unchanged |

At every stage the **topic names stay the same**. Only the publisher behind them changes (remaps allowed in launch).

## How viz gets data (edge mode)

Viz is **not** a ROS “sensor node” and **not** tied to StreamHub.

1. Something publishes `/imu/data`, `/lidar`, …
2. `viz_bus_server` runs `RosBusIngress` (ROS subscriptions)
3. Samples land in `TickBuffer` (same JSON shape as StreamHub ticks)
4. Browser opens `WS /ws` and renders

So when you plug in real sensors or play a bag onto those topics, **the UI keeps working** without UI changes.

## How to swap in real / recorded data later

### Bag playback

```bash
# terminal 1 — processor + viz only (no sim)
ros2 launch edge_sensor_sim edge_vehicle.launch.py use_sim:=false

# terminal 2 — your bag must contain (or remap to) bus topics
ros2 bag play your_drive.mcap --remap ...
```

### Real drivers

Point driver launch remaps at the bus:

```yaml
# example
lidar_driver:
  ros__parameters:
    frame_id: lidar_link
# launch remap: driver_cloud_topic → /lidar
```

### Sim stays useful

Keep `sensor_suite_node` as a **source adapter** for CI and desk dev. It is not the architecture center — the **bus** is.

## Package layout (logical)

See [`STRUCTURE.md`](STRUCTURE.md) for the full tree. Summary:

```text
edge_sensor_sim/
  domain/              # maps + pure models
  stream/              # sole generator (live/replay)
  bus/                 # topic contract + TickBuffer
  adapters/ros/        # sensor_source, processor, bus_ingress
  adapters/web/        # hub_server, bus_server, static UI
  adapters/files/      # record/offline CLIs
  apps/                # stream_server process
  launch/edge_vehicle.launch.py
```

Later you can split ROS packages without changing bus topic names.

## Dev vs edge quick reference

| Want | Command |
|---|---|
| Fast UI | `./start.sh` |
| Realistic edge path | `./start.sh --edge` |
| Host UI no Docker | `./scripts/run_viz.sh` |
| Record sim streams | `./scripts/generate_streams.sh` |
