#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_dir}/install/setup.bash"
export DRONE_SCENARIO_FILE="${DRONE_SCENARIO_FILE:-${repo_dir}/config/scenarios.yaml}"
export DRONE_ARTIFACT_DIR="${DRONE_ARTIFACT_DIR:-${repo_dir}/artifacts}"
ros2 launch drone_observer drone_observer.launch.py \
  scenario:="${DRONE_SCENARIO:-all}" \
  scenario_file:="${DRONE_SCENARIO_FILE}" \
  artifact_dir:="${DRONE_ARTIFACT_DIR}"
