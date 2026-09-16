#!/usr/bin/env bash
# Container entry for multi-package haul-edge stack.
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

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

# Editable multi-package path (before colcon install)
export PYTHONPATH="/project/ws/src/edge_sim:/project/ws/src/edge_sensor_source:/project/ws/src/edge_processor:/project/ws/src/edge_viz:/project/ws/src/edge_bringup${PYTHONPATH:+:$PYTHONPATH}"
cd /project

build_ros_pkgs() {
  echo "[entrypoint] Building packages (colcon)..."
  cd /project/ws
  colcon build --packages-select edge_sim edge_sensor_source edge_processor edge_viz edge_bringup --symlink-install
  set +u
  # shellcheck disable=SC1091
  source /project/ws/install/setup.bash
  set -u
  cd /project
}

hub_args=(
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
  hub_args+=(--stream "${STREAM_PATH}" --mode replay)
fi
if [[ -n "${RECORD_DIR}" ]]; then
  hub_args+=(--record-dir "${RECORD_DIR}")
fi

case "${MODE}" in
  shell)
    echo "[entrypoint] Interactive shell"
    echo "  Dev UI:  python3 -m edge_viz.run_from_hub"
    echo "  Edge:    ros2 launch edge_bringup edge_vehicle.launch.py"
    exec bash
    ;;
  edge)
    build_ros_pkgs
    echo "[entrypoint] Edge stack: sensor_source + processor + viz_from_bus"
    echo "[entrypoint] Browser → http://127.0.0.1:${VIZ_PORT}/"
    exec ros2 launch edge_bringup edge_vehicle.launch.py \
      use_sim:=true use_processor:=true use_viz:=true viz_port:="${VIZ_PORT}"
    ;;
  ros)
    build_ros_pkgs
    echo "[entrypoint] Sensor source only (bus publishers)"
    exec ros2 launch edge_bringup sensor_source.launch.py
    ;;
  all|viz+ros|ros+viz)
    build_ros_pkgs
    echo "[entrypoint] Dev: StreamHub viz + optional ROS source in-process"
    echo "[entrypoint] Browser → http://127.0.0.1:${VIZ_PORT}/"
    exec python3 -m edge_viz.run_from_hub "${hub_args[@]}" --ros
    ;;
  viz|*)
    echo "[entrypoint] Dev UI: StreamHub → viz (no ROS bus required)"
    echo "[entrypoint] Browser → http://127.0.0.1:${VIZ_PORT}/"
    exec python3 -m edge_viz.run_from_hub "${hub_args[@]}"
    ;;
esac
