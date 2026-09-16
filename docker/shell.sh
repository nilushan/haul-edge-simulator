#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
export COMPOSE_PROJECT_NAME="${COMPOSE_PROJECT_NAME:-haul-edge-sim}"
docker compose -f docker/docker-compose.yml build sim
exec docker compose -f docker/docker-compose.yml run --rm -e MODE=shell sim bash -lc 'set +u; source /opt/ros/jazzy/setup.bash; set -u; exec bash'
