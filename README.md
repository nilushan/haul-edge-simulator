# Haul edge sensor sim

Synthetic **haul-truck** on-vehicle sensor suite for local development:

| Stream | ROS 2 type | Topic (default) |
|---|---|---|
| Vehicle motion (shared truth) | — | drives all sensors |
| **IMU** | `sensor_msgs/Imu` | `/imu/data` |
| **GNSS** | `sensor_msgs/NavSatFix` | `/gnss/fix` |
| **LiDAR** | `sensor_msgs/PointCloud2` | `/lidar` |
| Ground-truth pose (debug) | `nav_msgs/Odometry` | `/sim/ground_truth/odom` |

**Design:** sensor models are **pure Python** (no ROS). A ROS 2 node only samples them and publishes. You can also generate offline files without ROS.

```text
haul-edge-sim/
├── README.md
├── docker/                 # optional Jazzy runner (macOS-friendly)
├── scripts/
│   └── generate_offline.py # write CSV/NPZ without ROS
├── data/                   # generated offline samples (gitignored bulk)
└── ws/src/edge_sensor_sim/ # ROS 2 package
    ├── edge_sensor_sim/
    │   ├── models/         # IMU / GNSS / LiDAR / vehicle
    │   └── sensor_suite_node.py
    ├── launch/
    └── config/
```

## Quick start (all-in-one Docker)

```bash
./start.sh
# → builds image, starts ~60 s looping sensor stream + web UI
# open http://127.0.0.1:8099/
```

| Command | What it runs |
|---|---|
| `./start.sh` | Visualizer (REST + WebSocket UI) |
| `./start.sh --all` | Visualizer **+** ROS 2 publishers |
| `./start.sh --ros` | ROS 2 `sensor_suite` only |
| `./start.sh --shell` | Interactive Jazzy shell |
| `./start.sh --build-only` | Build image and exit |

```bash
VIZ_PORT=8099 DURATION=60 ./start.sh --all
```

### Host-only visualizer (no Docker)

```bash
pip3 install -r requirements.txt
./scripts/run_viz.sh
# open http://127.0.0.1:8099/
```

### Manual Docker / ROS

```bash
./docker/build.sh
./docker/shell.sh
# inside:
source /opt/ros/jazzy/setup.bash
cd /project/ws && colcon build --packages-select edge_sensor_sim && source install/setup.bash
ros2 launch edge_sensor_sim sensor_suite.launch.py
```

```bash
ros2 topic hz /lidar /imu/data /gnss/fix
```

## Web visualizer API

Live charts + path + LiDAR BEV for a **~60 s** looping stream.

| API | Description |
|---|---|
| `GET /` | Dashboard UI |
| `GET /api/healthz` | Liveness |
| `GET /api/status` | Latest samples + ~1 min history tails |
| `GET /api/history/{imu\|gnss\|odom}` | Full ring-buffer history |
| `GET /api/lidar` | Latest downsampled cloud (BEV) |
| `GET /api/config` | Rates / duration |
| `WS /ws` | Live ticks (~10 Hz) with chart tails + lidar |

Env overrides: `PORT=8099` `DURATION=60` `HOST=127.0.0.1`

## Offline generation (no ROS)

```bash
python3 scripts/generate_offline.py --seconds 60 --out data/sample_run
# writes: vehicle.csv, imu.csv, gnss.csv, lidar meta + optional npz frames
```

## Native Ubuntu 24.04 + Jazzy

```bash
source /opt/ros/jazzy/setup.bash
cd ws && colcon build --packages-select edge_sensor_sim && source install/setup.bash
ros2 launch edge_sensor_sim sensor_suite.launch.py
```

## What is simulated (v1)

- **Vehicle:** constant-speed haul along a gentle curve + vertical undulations + occasional bumps (tyre hits).
- **IMU:** specific force + angular rate from motion, plus bias and Gaussian noise.
- **GNSS:** lat/lon/alt from local ENU origin, with horizontal noise and occasional dropouts.
- **LiDAR:** ring-style scan — ground plane, left/right berms, random rocks; ego motion spins the scene in world frame then expressed in `base_link`.

Not yet: cameras, Coral/ONNX, full LIO, or application-specific production logic (rock severity). Those can subscribe to these topics later.

## Frames

- `map` / ENU tangent at GNSS origin  
- `base_link` — vehicle body  
- `lidar_link`, `imu_link` — fixed offsets (TF static in launch)

## Next steps

1. Record `ros2 bag record /lidar /imu/data /gnss/fix`  
2. Point `lidar_ground_seg` at `/lidar`  
3. Add camera + detection nodes when ready  
