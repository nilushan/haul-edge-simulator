#!/usr/bin/env bash
# Start REST + WebSocket sensor visualizer (~60 s looping stream).
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

echo "Open http://${HOST}:${PORT}/"
exec python3 -m edge_sensor_sim.viz_server \
  --host "${HOST}" \
  --port "${PORT}" \
  --duration "${DURATION}" \
  --imu-hz 50 \
  --gnss-hz 5 \
  --lidar-hz 5 \
  --ws-hz 10
