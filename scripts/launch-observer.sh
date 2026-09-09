#!/usr/bin/env bash
set -euo pipefail

source /opt/drone/scripts/ros-env.sh
source /opt/drone_ws/install/setup.bash

ros2 launch drone_observer drone_observer.launch.py \
  scenario:="${DRONE_SCENARIO:-all}" \
  scenario_file:="${DRONE_SCENARIO_FILE:-/opt/drone-demo/config/scenarios.yaml}" \
  artifact_dir:="${DRONE_ARTIFACT_DIR:-/tmp/drone-artifacts}" \
  home_x:="${DRONE_HOME_X:-10.0}" \
  home_y:="${DRONE_HOME_Y:-16.0}" \
  takeoff_altitude:="${DRONE_TAKEOFF_ALTITUDE:-12.0}" \
  landing_altitude:="${DRONE_LANDING_ALTITUDE:-0.6}"
