#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/micromamba/envs/ros_env/setup.bash
if [ -f /opt/drone_ws/install/setup.bash ]; then
  source /opt/drone_ws/install/setup.bash
fi
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ZENOH_ROUTER_ENDPOINT="${ZENOH_ROUTER_ENDPOINT:-tcp/localhost:7447}"
export ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"${ZENOH_ROUTER_ENDPOINT}\"]"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/opt/drone/.ros/log}"
export HOME="${HOME:-/opt/drone}"
