# Libraries

Pure Python packages — **no ROS nodes**.

| Package | Purpose |
|---|---|
| `edge_sim` | Maps, sensor models, StreamHub, bus topic contract |
| `edge_perception` | Ground / rocks / bunds / vibration algorithms + alert schemas |

Import and unit-test without a ROS daemon:

```bash
source scripts/env_pythonpath.sh
python3 -c 'from edge_perception.rocks import detect_rocks'
```
