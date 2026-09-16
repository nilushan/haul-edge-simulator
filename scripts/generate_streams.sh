#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
# shellcheck disable=SC1091
source "${ROOT}/scripts/env_pythonpath.sh"
SECONDS_LEN="${SECONDS_LEN:-60}"
OUT="${OUT:-${ROOT}/data/streams}"
python3 -m edge_sim.tools.generate_stream \
  --maps haul_corridor,tight_switchbacks,open_pit_bench,rocky_descent \
  --seconds "${SECONDS_LEN}" \
  --out-root "${OUT}"
