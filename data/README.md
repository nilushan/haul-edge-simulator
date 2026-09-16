# Data

| Path | Purpose |
|---|---|
| `streams/` | Replayable multi-map sensor streams (sole generator format) |

## Streams layout

```text
data/streams/
  index.json
  haul_corridor/
    manifest.json
    odom.jsonl
    imu.jsonl
    gnss.jsonl
    lidar.jsonl
  …
```

Generate:

```bash
./scripts/generate_streams.sh
# or
python3 -m edge_sensor_sim.adapters.files.generate_stream --seconds 60 --out-root data/streams
```

Replay:

```bash
STREAM_MODE=replay ./scripts/run_viz.sh
# or
./start.sh --replay
```

Bulk stream files are gitignored (`data/**`); keep this README tracked.
