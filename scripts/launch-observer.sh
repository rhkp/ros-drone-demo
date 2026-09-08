#!/usr/bin/env bash
set -euo pipefail

source /opt/drone/scripts/ros-env.sh
source /opt/drone_ws/install/setup.bash

ros2 launch drone_observer drone_observer.launch.py \
  scenario:="${DRONE_SCENARIO:-all}" \
  scenario_file:="${DRONE_SCENARIO_FILE:-/opt/drone-demo/config/scenarios.yaml}" \
  artifact_dir:="${DRONE_ARTIFACT_DIR:-/tmp/drone-artifacts}"
