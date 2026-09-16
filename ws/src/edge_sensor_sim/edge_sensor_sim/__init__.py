"""
Haul-edge vehicle sensor stack.

Layout
------
domain/     Pure sim models + map presets (no ROS / no HTTP)
stream/     Sole sensor-stream generator (live + replay)
bus/        Stable edge topic contract + tick buffer
adapters/   Technology edges
  ros/      ROS publishers / subscribers / processors
  web/      Browser visualizer (hub-direct or bus-subscribed)
  files/    Offline / stream recording CLIs
apps/       Process entrypoints (compose adapters)
"""

__version__ = '0.2.0'
