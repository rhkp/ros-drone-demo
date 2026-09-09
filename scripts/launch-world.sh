#!/usr/bin/env bash
set -euo pipefail

source /opt/rmf/scripts/ros-env.sh
WORLD_FILE="${DRONE_WORLD_FILE:-/opt/drone-demo/worlds/farm_survey.sdf}"
WORLD_NAME="${DRONE_WORLD_NAME:-farm_survey}"
export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"${ZENOH_ROUTER_ENDPOINT}\"]"

gz_args=(-r)
if [[ "${DRONE_GUI:-false}" != "true" ]]; then
  gz_args+=(-s)
fi

gz sim "${gz_args[@]}" "${WORLD_FILE}" &
gz_pid=$!

sleep 2
ros2 run ros_gz_bridge parameter_bridge \
  "/drone/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/drone/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
  "/world/${WORLD_NAME}/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean" &
bridge_pid=$!

cleanup() {
  kill "${bridge_pid}" "${gz_pid}" 2>/dev/null || true
}
trap cleanup EXIT

wait "${gz_pid}"
