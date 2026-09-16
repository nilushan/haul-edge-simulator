#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
export COMPOSE_PROJECT_NAME="${COMPOSE_PROJECT_NAME:-haul-edge-sim}"
docker compose -f docker/docker-compose.yml build sim
docker compose -f docker/docker-compose.yml run --rm sim bash -lc '
  set -eo pipefail
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  cd /project/ws
  colcon build --packages-select edge_sensor_sim
  set +u
  source install/setup.bash
  set -u
  ros2 launch edge_sensor_sim sensor_suite.launch.py
'
