#!/usr/bin/env bash
# All-in-one: build Docker image and run haul-edge-sim stack.
#
# Usage:
#   ./start.sh              # visualizer only  → http://127.0.0.1:8099/
#   ./start.sh --ros        # ROS publishers only
#   ./start.sh --all        # ROS + visualizer
#   ./start.sh --shell      # interactive Jazzy shell
#   ./start.sh --build-only # build image and exit
#
# Env:
#   VIZ_PORT=8099   DURATION=60   MODE=viz|ros|all|shell
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

# Unique compose project name — compose file lives in docker/, which would
# otherwise default the project to "docker" and collide with other projects.
export COMPOSE_PROJECT_NAME="${COMPOSE_PROJECT_NAME:-haul-edge-sim}"

MODE="${MODE:-viz}"
VIZ_PORT="${VIZ_PORT:-8099}"
DURATION="${DURATION:-60}"
BUILD_ONLY=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --viz) MODE=viz ;;
    --ros) MODE=ros ;;
    --all|--viz-ros) MODE=all ;;
    --shell) MODE=shell ;;
    --build-only) BUILD_ONLY=1 ;;
    --port) VIZ_PORT="${2:?}"; shift ;;
    --duration) DURATION="${2:?}"; shift ;;
    -h|--help)
      sed -n '2,16p' "$0" | tr -d '#'
      exit 0
      ;;
    *)
      echo "Unknown option: $1 (try --help)" >&2
      exit 1
      ;;
  esac
  shift
done

export MODE VIZ_PORT DURATION

if ! command -v docker >/dev/null 2>&1; then
  echo "error: docker not found. For host-only viz: ./scripts/run_viz.sh" >&2
  exit 1
fi

echo "============================================================"
echo " haul-edge-sim"
echo "   mode:     ${MODE}"
echo "   duration: ${DURATION}s (looping stream)"
echo "   viz port: ${VIZ_PORT}"
echo "============================================================"

echo "[start] Building image haul-edge-sim:jazzy ..."
docker compose -f docker/docker-compose.yml build sim

if [[ "${BUILD_ONLY}" -eq 1 ]]; then
  echo "[start] Build complete."
  exit 0
fi

# Ensure entrypoint is executable on the mounted volume
chmod +x docker/entrypoint.sh 2>/dev/null || true

if [[ "${MODE}" == "shell" ]]; then
  echo "[start] Interactive shell (Ctrl+D to exit)"
  exec docker compose -f docker/docker-compose.yml run --rm \
    -e MODE=shell \
    sim bash -lc 'source /opt/ros/jazzy/setup.bash; exec bash'
fi

echo "[start] Starting container (Ctrl+C to stop)"
if [[ "${MODE}" == "viz" || "${MODE}" == "all" ]]; then
  echo "[start] Browser UI → http://127.0.0.1:${VIZ_PORT}/"
fi
if [[ "${MODE}" == "ros" || "${MODE}" == "all" ]]; then
  echo "[start] ROS topics: /lidar /imu/data /gnss/fix /sim/ground_truth/odom"
fi

exec docker compose -f docker/docker-compose.yml run --rm --service-ports \
  -e MODE="${MODE}" \
  -e DURATION="${DURATION}" \
  -e VIZ_HOST=0.0.0.0 \
  -e VIZ_PORT=8099 \
  sim bash /project/docker/entrypoint.sh
