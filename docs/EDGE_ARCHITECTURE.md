# Vehicle edge architecture

## Packages

| Package | Role |
|---|---|
| `edge_sim` | Fake sensors + multi-map StreamHub + topic name contract |
| `edge_sensor_source` | ROS node: publish sim/replay onto bus topics |
| `edge_processor` | ROS node: example processing on the bus |
| `edge_viz` | Browser UI (hub mode or bus mode) |
| `edge_bringup` | Launch files + shared config |

## Bus topics (`edge_sim.topics`)

| Topic | Type |
|---|---|
| `/imu/data` | `sensor_msgs/Imu` |
| `/gnss/fix` | `sensor_msgs/NavSatFix` |
| `/lidar` | `sensor_msgs/PointCloud2` |
| `/odom` | `nav_msgs/Odometry` |
| `/edge/lidar/processed` | `PointCloud2` |
| `/edge/status` | `std_msgs/String` JSON |

## Gradual realism

1. **Now:** `edge_sensor_source` (sim) on the bus  
2. **Next:** `ros2 bag play` with remaps, `use_sim:=false`  
3. **Then:** real driver packages publishing the same names  
4. **Processors:** add packages beside `edge_processor`  

Viz in bus mode (`edge_viz.run_from_bus`) never cares which source is upstream.

## Diagram

```text
 edge_sim                edge_sensor_source
 models/stream    ──►    publish bus topics
                                │
              ┌─────────────────┼─────────────────┐
              ▼                 ▼                 ▼
       edge_processor    edge_viz.bus_ingress   other nodes
              │                 │
              ▼                 ▼
         /edge/*          browser (WS)
```
