#!/usr/bin/env bash
set -euo pipefail

source /opt/rmf/scripts/ros-env.sh
WORLD_FILE="${DRONE_WORLD_FILE:-/opt/drone-demo/worlds/farm_survey.sdf}"

gz sim -r -s "${WORLD_FILE}" &
gz_pid=$!
trap 'kill "${gz_pid}" 2>/dev/null || true' EXIT

wait "${gz_pid}"
