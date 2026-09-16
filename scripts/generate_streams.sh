#!/usr/bin/env bash
# Pre-record replayable streams for all map presets into data/streams/.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
export PYTHONPATH="${ROOT}/ws/src/edge_sensor_sim${PYTHONPATH:+:$PYTHONPATH}"

SECONDS_LEN="${SECONDS_LEN:-60}"
OUT="${OUT:-${ROOT}/data/streams}"

python3 -m edge_sensor_sim.generate_stream \
  --maps haul_corridor,tight_switchbacks,open_pit_bench,rocky_descent \
  --seconds "${SECONDS_LEN}" \
  --out-root "${OUT}"
