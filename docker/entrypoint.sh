#!/usr/bin/env bash
# Container entry:
#   viz/all/ros  → StreamHub (+ optional ROS)
#   edge         → realistic bus path: source → processor → viz_bus
set -euo pipefail

MODE="${MODE:-viz}"
DURATION="${DURATION:-60}"
VIZ_HOST="${VIZ_HOST:-0.0.0.0}"
VIZ_PORT="${VIZ_PORT:-8099}"
STREAM_MODE="${STREAM_MODE:-live}"
MAPS="${MAPS:-haul_corridor,tight_switchbacks,open_pit_bench,rocky_descent}"
STREAM_PATH="${STREAM_PATH:-}"
STREAMS_ROOT="${STREAMS_ROOT:-/project/data/streams}"
RECORD_DIR="${RECORD_DIR:-}"

# ROS setup.bash references optional unset vars; incompatible with bash nounset.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

export PYTHONPATH="/project/ws/src/edge_sensor_sim${PYTHONPATH:+:$PYTHONPATH}"
cd /project

build_ros_pkg() {
  echo "[entrypoint] Building edge_sensor_sim (colcon)..."
  cd /project/ws
  colcon build --packages-select edge_sensor_sim --symlink-install
  set +u
  # shellcheck disable=SC1091
  source /project/ws/install/setup.bash
  set -u
  cd /project
}

stream_args=(
  --host "${VIZ_HOST}"
  --port "${VIZ_PORT}"
  --duration "${DURATION}"
  --mode "${STREAM_MODE}"
  --maps "${MAPS}"
  --streams-root "${STREAMS_ROOT}"
  --imu-hz 50
  --gnss-hz 5
  --lidar-hz 5
  --ws-hz 10
)

if [[ -n "${STREAM_PATH}" ]]; then
  stream_args+=(--stream "${STREAM_PATH}" --mode replay)
fi
if [[ -n "${RECORD_DIR}" ]]; then
  stream_args+=(--record-dir "${RECORD_DIR}")
fi

case "${MODE}" in
  shell)
    echo "[entrypoint] Interactive shell."
    echo "  Dev UI:   python3 -m edge_sensor_sim.apps.stream_server"
    echo "  Edge bus: ros2 launch edge_sensor_sim edge_vehicle.launch.py"
    exec bash
    ;;
  edge)
    # Realistic path: sources publish bus topics; processor + viz only subscribe.
    build_ros_pkg
    echo "[entrypoint] Edge vehicle stack (bus): sim source → processor → viz_bus"
    echo "[entrypoint] Browser UI → http://127.0.0.1:${VIZ_PORT}/"
    exec ros2 launch edge_sensor_sim edge_vehicle.launch.py \
      use_sim:=true use_processor:=true use_viz:=true viz_port:="${VIZ_PORT}"
    ;;
  ros)
    build_ros_pkg
    echo "[entrypoint] Sole StreamHub → ROS publishers only"
    exec python3 -m edge_sensor_sim.apps.stream_server "${stream_args[@]}" --ros --no-viz
    ;;
  all|viz+ros|ros+viz)
    build_ros_pkg
    echo "[entrypoint] Sole StreamHub → viz + ROS (shared hub, dev shortcut)"
    echo "[entrypoint] Browser UI → http://127.0.0.1:${VIZ_PORT}/"
    exec python3 -m edge_sensor_sim.apps.stream_server "${stream_args[@]}" --ros
    ;;
  viz|*)
    echo "[entrypoint] Dev path: StreamHub → visualizer (no ROS bus)"
    echo "[entrypoint] Open http://127.0.0.1:${VIZ_PORT}/  | edge mode: MODE=edge"
    exec python3 -m edge_sensor_sim.apps.stream_server "${stream_args[@]}"
    ;;
esac
