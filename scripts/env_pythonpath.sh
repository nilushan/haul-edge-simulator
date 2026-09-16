#!/usr/bin/env bash
# Build PYTHONPATH for editable (non-colcon) runs of libs + nodes.
# Usage:  source scripts/env_pythonpath.sh
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
paths=()
for d in "${ROOT}/ws/src/libs"/* "${ROOT}/ws/src/nodes"/* "${ROOT}/ws/src/bringup"/*; do
  [[ -d "$d" ]] || continue
  paths+=("$d")
done
export HAUL_EDGE_ROOT="$ROOT"
export PYTHONPATH="$(IFS=:; echo "${paths[*]}")${PYTHONPATH:+:$PYTHONPATH}"
