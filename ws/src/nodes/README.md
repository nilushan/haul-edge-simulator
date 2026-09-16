# Nodes

ROS 2 packages — thin wrappers around `libs/`.

| Package | Subscribes | Publishes |
|---|---|---|
| `edge_sensor_source` | (owns StreamHub) | `/imu` `/gnss` `/lidar` `/odom` |
| `edge_processor` | `/lidar` | `/edge/lidar/processed` |
| `edge_rock_detect` | `/lidar` | `/edge/lidar/{ground,obstacles,rocks}`, `/edge/detections`, `/edge/alerts` |
| `edge_bund_detect` | `/lidar` | `/edge/lidar/bunds`, `/edge/detections`, `/edge/alerts` |
| `edge_vibe_detect` | `/imu/data`, `/odom` | `/edge/vibe/features`, `/edge/alerts` |
| `edge_event_store` | `/edge/alerts` | SQLite + `/edge/status` |
| `edge_viz` | hub or bus | browser WebSocket |

Launch via `edge_bringup` (see `ws/src/bringup/`).
