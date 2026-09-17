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
| `edge_perception` | lib | Frame classification (road/ground/bund/rock/non-ground), events, class + event style table |
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
| `/edge/lidar/semantic` | `PointCloud2` | rock detect → (whole frame, per-point class) |
| `/edge/lidar/ground` | `PointCloud2` | rock detect → |
| `/edge/lidar/obstacles` | `PointCloud2` | rock detect → |
| `/edge/lidar/rocks` | `PointCloud2` | rock detect → |
| `/edge/lidar/bunds` | `PointCloud2` | bund detect → |
| `/edge/detections` | `std_msgs/String` JSON | detectors → |
| `/edge/alerts` | `std_msgs/String` JSON | detectors → |
| `/edge/vibe/features` | `std_msgs/String` JSON | vibe → |
| `/edge/status` | `std_msgs/String` JSON | all nodes → |

Detection clouds use fields: `x y z intensity label conf`, where `label` is an
`edge_perception.schema.Label` value.

## Detection classes and events

One classifier, `edge_perception.semantics.classify_frame`, produces every class
and every event in the stack. Both detector nodes and the inline viz pipeline
call it, so the bus and the browser never disagree about what a point is.

| Class | `Label` | Meaning |
|---|---|---|
| `road` | 4 | Drivable corridor surface (corridor width is measured per frame) |
| `ground` | 0 | Traversable surface outside the corridor |
| `bund` | 2 | Berm crest at or above the compliance height |
| `bund_low` | 5 | Berm crest below the compliance height |
| `cut_slope` | 6 | Batter climbing away from the road on the high side |
| `rock` | 1 | Compact non-ground cluster of rock-like size |
| `obstacle` | 3 | Non-ground return that is not a rock or a bund |
| `unknown` | 255 | No ground support in the cell, so no class was assigned |

| Event | Severity drivers | Payload highlights |
|---|---|---|
| `rock` | size, lateral offset, range | `radius_m`, `range_m`, `in_lane` |
| `bund_low` | measured height vs required | `min_height_m`, `spec_height_m`, `deficit_m`, `length_m`, `side` |
| `bund_gap` | gap length | `length_m`, `side` |
| `excessive_vibration` | RMS / peak vertical accel | `rms_az`, `peak_az`, `speed_mps` |

Colours, titles, and short map tags live in one table
(`edge_perception.schema.style_catalog`) served to the browser at
`/api/classes`. Add a class or event there and the 3D view, the toggles, the
legend, and the event feed all pick it up — no colours are hard-coded in the UI.

### The corridor is measured, not assumed

A haul road runs three to four trucks wide, varies in width, and curves out of
the body-frame band within the look-ahead; the truck sits in a lane, so its two
shoulders are at very different distances. The classifier therefore measures
**each edge separately and per longitudinal bin**, taking the innermost clearly
raised return as that bin's edge and smoothing across neighbouring bins so a
rock near the shoulder cannot pull the edge inwards.

Each shoulder is then typed. If the ground keeps climbing past where a berm
would sit, that side is a **cut batter**: it gets its own class and raises no
bund findings, because nothing has to stop a truck falling uphill.

### Measurement honesty

- Heights are measured against a **plane fitted to the running surface**, not
  against a constant per-bin height. A couple of degrees of vehicle roll lifts
  one side of a 30 m road by most of a metre in the body frame — more than a
  bund is tall — so a flat reference reads the far shoulder as raised and the
  near one as sunken. The fit seeds from the lane the truck is in and grows
  over the points that agree with it.
- Bund height is measured against that surface, not against the local ground
  grid (inside a berm the grid rides up with the berm and would report a few
  centimetres).
- A crest height is only claimed when the scan reached **past the crest and
  came back down** the far side. Along a distant shoulder the beams skim over
  the crest and stop on the inner flank, which would otherwise read as a low
  bund on every frame. In practice this limits height findings to the shoulder
  the truck is driving beside — the far shoulder is reported as a bund with no
  height claim.
- An unsampled shoulder only counts as a gap within `gap_observed_max_x_m`
  longitudinally and `gap_max_edge_m` laterally; beyond that, absence of
  returns is about grazing geometry, not a missing bund.
- Rock detection on a 16-ring scan is range-limited: reliable from roughly
  20 m in, median first detection around 12 m.

## Gradual realism

1. **Now:** `edge_sensor_source` (sim) on the bus  
2. **Next:** `ros2 bag play` with remaps, `use_sim:=false`  
3. **Then:** real driver packages publishing the same names  
4. **Processors:** libs stay; only node wiring changes  

Viz in bus mode (`edge_viz.run_from_bus`) never cares which source is upstream.

In hub mode there is no bus, so `edge_viz.perception_overlay.PerceptionOverlay`
wraps the hub and runs the same classifier in-process — the browser sees the
same classes and events without ROS. Run with `--no-inline-detect` to serve raw
hub ticks instead.

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
