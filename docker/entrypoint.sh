#!/usr/bin/env bash
# Container entry: start visualizer and optionally ROS sensor suite.
set -euo pipefail

MODE="${MODE:-viz}"
DURATION="${DURATION:-60}"
VIZ_HOST="${VIZ_HOST:-0.0.0.0}"
VIZ_PORT="${VIZ_PORT:-8099}"

# ROS setup.bash references optional unset vars; incompatible with bash nounset.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

export PYTHONPATH="/project/ws/src/edge_sensor_sim${PYTHONPATH:+:$PYTHONPATH}"
cd /project

ROS_PID=""
cleanup() {
  if [[ -n "${ROS_PID}" ]] && kill -0 "${ROS_PID}" 2>/dev/null; then
    kill "${ROS_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

start_ros() {
  echo "[entrypoint] Building edge_sensor_sim (colcon)..."
  cd /project/ws
  colcon build --packages-select edge_sensor_sim --symlink-install
  set +u
  # shellcheck disable=SC1091
  source /project/ws/install/setup.bash
  set -u
  cd /project
  echo "[entrypoint] Launching ROS 2 sensor_suite_node..."
  ros2 launch edge_sensor_sim sensor_suite.launch.py &
  ROS_PID=$!
}

start_viz() {
  echo "[entrypoint] Starting viz server on ${VIZ_HOST}:${VIZ_PORT} (duration=${DURATION}s loop)"
  echo "[entrypoint] Open http://127.0.0.1:${VIZ_PORT:-8099}/ on the host"
  exec python3 -m edge_sensor_sim.viz_server \
    --host "${VIZ_HOST}" \
    --port "${VIZ_PORT}" \
    --duration "${DURATION}" \
    --imu-hz 50 \
    --gnss-hz 5 \
    --lidar-hz 5 \
    --ws-hz 10
}

case "${MODE}" in
  shell)
    echo "[entrypoint] Interactive shell. Try: ./scripts/run_viz.sh  or  ros2 launch ..."
    exec bash
    ;;
  ros)
    start_ros
    # keep container alive on ROS only
    wait "${ROS_PID}"
    ;;
  all|viz+ros|ros+viz)
    start_ros
    start_viz
    ;;
  viz|*)
    start_viz
    ;;
esac
