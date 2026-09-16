#!/usr/bin/env bash
# All-in-one: build Docker image and run haul-edge-sim stack.
#
# Usage:
#   ./start.sh              # dev: StreamHub → viz (no ROS bus)
#   ./start.sh --edge       # realistic: source → bus → processor → viz_bus
#   ./start.sh --ros        # StreamHub → ROS publishers only
#   ./start.sh --all        # StreamHub → ROS + direct viz (dev shortcut)
#   ./start.sh --replay     # replay data/streams playlist (dev path)
#   ./start.sh --shell      # interactive Jazzy shell
#   ./start.sh --build-only # build image and exit
#
# Env:
#   VIZ_PORT=8099  DURATION=60  MODE=viz|edge|ros|all|shell
#   STREAM_MODE=live|replay  MAPS=id,id  STREAM_PATH=/path  STREAMS_ROOT=data/streams
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

# Unique compose project name — compose file lives in docker/, which would
# otherwise default the project to "docker" and collide with other projects.
export COMPOSE_PROJECT_NAME="${COMPOSE_PROJECT_NAME:-haul-edge-sim}"

MODE="${MODE:-viz}"
VIZ_PORT="${VIZ_PORT:-8099}"
DURATION="${DURATION:-60}"
STREAM_MODE="${STREAM_MODE:-live}"
MAPS="${MAPS:-haul_corridor,tight_switchbacks,open_pit_bench,rocky_descent}"
STREAMS_ROOT="${STREAMS_ROOT:-/project/data/streams}"
STREAM_PATH="${STREAM_PATH:-}"
BUILD_ONLY=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --viz) MODE=viz ;;
    --edge) MODE=edge ;;
    --ros) MODE=ros ;;
    --all|--viz-ros) MODE=all ;;
    --shell) MODE=shell ;;
    --build-only) BUILD_ONLY=1 ;;
    --replay) STREAM_MODE=replay ;;
    --live) STREAM_MODE=live ;;
    --maps) MAPS="${2:?}"; shift ;;
    --stream) STREAM_PATH="${2:?}"; STREAM_MODE=replay; shift ;;
    --port) VIZ_PORT="${2:?}"; shift ;;
    --duration) DURATION="${2:?}"; shift ;;
    -h|--help)
      sed -n '2,20p' "$0" | tr -d '#'
      exit 0
      ;;
    *)
      echo "Unknown option: $1 (try --help)" >&2
      exit 1
      ;;
  esac
  shift
done

export MODE VIZ_PORT DURATION STREAM_MODE MAPS STREAMS_ROOT STREAM_PATH

if ! command -v docker >/dev/null 2>&1; then
  echo "error: docker not found. For host-only viz: ./scripts/run_viz.sh" >&2
  exit 1
fi

echo "============================================================"
echo " haul-edge-sim"
echo "   mode:        ${MODE}"
echo "   stream:      ${STREAM_MODE}"
echo "   maps:        ${MAPS}"
echo "   duration:    ${DURATION}s / map (loop + cycle)"
echo "   viz port:    ${VIZ_PORT}"
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
if [[ "${MODE}" == "viz" || "${MODE}" == "all" || "${MODE}" == "edge" ]]; then
  echo "[start] Browser UI → http://127.0.0.1:${VIZ_PORT}/"
fi
if [[ "${MODE}" == "ros" || "${MODE}" == "all" || "${MODE}" == "edge" ]]; then
  echo "[start] Bus topics: /imu/data /gnss/fix /lidar /odom"
fi
if [[ "${MODE}" == "edge" ]]; then
  echo "[start] Edge path: sim → bus → processor_stub → viz_bus"
fi

exec docker compose -f docker/docker-compose.yml run --rm --service-ports \
  -e MODE="${MODE}" \
  -e DURATION="${DURATION}" \
  -e STREAM_MODE="${STREAM_MODE}" \
  -e MAPS="${MAPS}" \
  -e STREAMS_ROOT="${STREAMS_ROOT}" \
  -e STREAM_PATH="${STREAM_PATH}" \
  -e VIZ_HOST=0.0.0.0 \
  -e VIZ_PORT=8099 \
  sim bash /project/docker/entrypoint.sh
