#!/usr/bin/env bash
set -euo pipefail

source /opt/rmf/scripts/ros-env.sh
WORLD_FILE="${DRONE_WORLD_FILE:-/opt/drone-demo/worlds/farm_survey.sdf}"
WORLD_NAME="${DRONE_WORLD_NAME:-farm_survey}"
export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"${ZENOH_ROUTER_ENDPOINT}\"]"

openbox_pid=""
if [[ "${DRONE_GUI:-false}" == "true" ]]; then
  export DISPLAY="${DISPLAY:-:99}"
  mkdir -p "${HOME}/.config/openbox"
  cat > "${HOME}/.config/openbox/rc.xml" <<'OBCONF'
<?xml version="1.0" encoding="UTF-8"?>
<openbox_config xmlns="http://openbox.org/3.4/rc">
  <theme>
    <name>Clearlooks</name>
    <titleLayout>NLIMC</titleLayout>
  </theme>
  <desktops><number>1</number></desktops>
  <resize><drawContents>yes</drawContents></resize>
  <applications>
    <application class="*"><decor>yes</decor></application>
  </applications>
</openbox_config>
OBCONF

  echo "[drone-world] Starting openbox window manager on ${DISPLAY}..."
  openbox &
  openbox_pid=$!
  sleep 1
fi

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
  kill "${openbox_pid:-}" 2>/dev/null || true
}
trap cleanup EXIT

wait "${gz_pid}"
