# Code structure

```text
haul-edge-sim/
├── start.sh                 # Docker entry (dev / edge / ros)
├── scripts/                 # Host helpers
├── docker/                  # Image + compose
├── data/streams/            # Replayable recorded streams
├── docs/                    # Architecture notes
└── ws/src/edge_sensor_sim/  # ROS 2 Python package
    ├── config/              # ROS params (bus topic names, rates)
    ├── launch/              # edge_vehicle, sensor_suite
    ├── test/
    └── edge_sensor_sim/     # Python library
        ├── domain/          # pure business/sim core (no ROS/HTTP)
        │   ├── maps.py      # map presets
        │   └── models/      # world, vehicle, imu, gnss, lidar
        ├── stream/          # sole stream generator (live + replay)
        │   ├── hub.py
        │   └── format.py    # JSONL record/replay
        ├── bus/             # edge topic contract + tick buffer
        │   ├── contract.py
        │   └── tick_buffer.py
        ├── adapters/        # technology edges
        │   ├── ros/         # publish/subscribe ROS bus
        │   ├── web/         # browser UI (static + servers)
        │   └── files/       # offline / stream CLIs
        └── apps/            # process composition (stream_server)
```

## Dependency direction

```text
adapters  ──uses──►  stream / bus  ──uses──►  domain
apps      ──wires──►  adapters + stream
```

- **domain** never imports ROS, aiohttp, or adapters
- **stream** only uses domain (generates sensor samples)
- **bus** is transport-agnostic contract + in-memory buffer
- **adapters.ros** turns hub samples into ROS topics, or ROS topics into ticks
- **adapters.web** only *reads* a tick source (hub or bus buffer)
- **apps** starts processes

## What lives where

| Want to change… | Look in |
|---|---|
| Map geometry / rocks | `domain/maps.py`, `domain/models/world.py` |
| IMU/GNSS/LiDAR math | `domain/models/` |
| Live rates, replay, multi-map cycle | `stream/hub.py` |
| Topic names (`/lidar`, …) | `bus/contract.py`, `config/` |
| ROS publisher node | `adapters/ros/sensor_source_node.py` |
| Example processor | `adapters/ros/processor_stub_node.py` |
| Browser UI | `adapters/web/static/` |
| Dev viz (hub → WS) | `adapters/web/hub_server.py` |
| Edge viz (bus → WS) | `adapters/web/bus_server.py` |
| `./start.sh` process wiring | `apps/stream_server.py`, `docker/entrypoint.sh` |

## Entrypoints

```bash
python3 -m edge_sensor_sim.apps.stream_server
python3 -m edge_sensor_sim.adapters.web.hub_server
python3 -m edge_sensor_sim.adapters.web.bus_server
python3 -m edge_sensor_sim.adapters.files.generate_stream
python3 -m edge_sensor_sim.adapters.files.generate_offline
```
