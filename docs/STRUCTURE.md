# Code structure

Readable layout: **sim / separate ROS nodes / viz / bringup**.

```text
ws/src/
├── edge_sim/                 # library only (no ROS nodes)
│   └── edge_sim/
│       ├── maps.py
│       ├── models/           # world, vehicle, imu, gnss, lidar
│       ├── stream/           # StreamHub + JSONL format
│       ├── topics.py         # shared topic name contract
│       └── tools/            # generate_stream / generate_offline
│
├── edge_sensor_source/       # ROS node package
│   └── sensor_source_node    # StreamHub → /imu /gnss /lidar /odom
│
├── edge_processor/           # ROS node package
│   └── processor_node        # /lidar → /edge/lidar/processed
│
├── edge_viz/                 # web UI package
│   └── edge_viz/
│       ├── app.py            # HTTP + WebSocket server
│       ├── run_from_hub.py   # DEV: read StreamHub
│       ├── run_from_bus.py   # EDGE: subscribe ROS topics
│       ├── bus_ingress.py    # ROS subscriptions for viz
│       ├── tick_buffer.py
│       └── static/           # browser UI
│
└── edge_bringup/             # launch + config only
    ├── launch/edge_vehicle.launch.py
    └── config/
```

## Why separate ROS packages?

Each node is its own package so you can:

- run / replace one node without touching others  
- swap `edge_sensor_source` for real drivers or bag play  
- grow processors (`edge_ground_seg`, `edge_detect`, …) as new packages  

## Data flow

```text
DEV
  edge_sim.StreamHub ──► edge_viz.run_from_hub ──► browser

EDGE
  edge_sensor_source ──publish──► ROS topics
  edge_processor     ──subscribe/publish──► /edge/*
  edge_viz.run_from_bus ──subscribe──► browser
```

## Commands

```bash
# Dev UI (no ROS)
./scripts/run_viz.sh
python3 -m edge_viz.run_from_hub

# Edge stack
ros2 launch edge_bringup edge_vehicle.launch.py

# Source only
ros2 launch edge_bringup sensor_source.launch.py
```
