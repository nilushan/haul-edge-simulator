#!/usr/bin/env bash
# Start sole StreamHub + visualizer subscriber (~60 s multi-map looping stream).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

if ! python3 -c 'import aiohttp' 2>/dev/null; then
  echo "Installing aiohttp..."
  pip3 install --user -q aiohttp || pip3 install -q aiohttp
fi

export PYTHONPATH="${ROOT}/ws/src/edge_sensor_sim${PYTHONPATH:+:$PYTHONPATH}"
HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-8099}"
DURATION="${DURATION:-60}"
STREAM_MODE="${STREAM_MODE:-live}"
MAPS="${MAPS:-haul_corridor,tight_switchbacks,open_pit_bench,rocky_descent}"
STREAMS_ROOT="${STREAMS_ROOT:-${ROOT}/data/streams}"

echo "Open http://${HOST}:${PORT}/  (StreamHub mode=${STREAM_MODE})"
exec python3 -m edge_sensor_sim.stream_server \
  --host "${HOST}" \
  --port "${PORT}" \
  --duration "${DURATION}" \
  --mode "${STREAM_MODE}" \
  --maps "${MAPS}" \
  --streams-root "${STREAMS_ROOT}" \
  ${STREAM_PATH:+--stream "${STREAM_PATH}"} \
  --imu-hz 50 \
  --gnss-hz 5 \
  --lidar-hz 5 \
  --ws-hz 10
