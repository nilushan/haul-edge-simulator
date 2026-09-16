#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
export COMPOSE_PROJECT_NAME="${COMPOSE_PROJECT_NAME:-haul-edge-sim}"
docker compose -f docker/docker-compose.yml build sim
docker compose -f docker/docker-compose.yml run --rm \
  -e MODE=ros \
  -e STREAM_MODE="${STREAM_MODE:-live}" \
  -e MAPS="${MAPS:-haul_corridor,tight_switchbacks,open_pit_bench,rocky_descent}" \
  -e DURATION="${DURATION:-600}" \
  sim bash /project/docker/entrypoint.sh
