# Roadmap

The simulator currently provides deterministic synthetic terrain, synchronized
sensor streams, ROS 2 adapters, perception events, local persistence, and a live
browser visualization.

## Near term

- Add benchmark fixtures with expected detection metrics for every map.
- Export recorded scenarios as MCAP in addition to JSONL.
- Add configurable sensor degradation such as dropped packets, GNSS drift, and
  partial LiDAR occlusion.
- Improve visualization accessibility and mobile layouts.

## Future exploration

- Support external point-cloud terrain through a documented import pipeline.
- Add camera simulation and image-based detections.
- Evaluate hardware-accelerated inference adapters without coupling them to the
  core perception library.
- Add an optional synchronization adapter for forwarding locally stored events
  when connectivity returns.

## Non-goals

- Modeling a specific mine or proprietary vehicle platform.
- Replacing validation with real sensors and operational safety testing.
- Treating synthetic detections as production safety decisions.
