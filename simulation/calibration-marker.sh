#!/bin/bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/create3_ws/install/setup.bash
set -u

action="${1:-}"
x="${2:-0.0}"
y="${3:-0.0}"
world="${SIM_WORLD:-room_camera}"
name=sim_camera_calibration_marker
model_file=/opt/pai/models/calibration_marker/model.sdf

remove_marker() {
  gz service -s "/world/${world}/remove" \
    --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 \
    --req "name: \"${name}\", type: MODEL" >/dev/null 2>&1 || true
}

case "$action" in
  move)
    remove_marker
    ros2 run ros_gz_sim create -world "$world" -file "$model_file" \
      -name "$name" -x "$x" -y "$y" -z 0.0 >/dev/null
    ;;
  remove)
    remove_marker
    ;;
  *)
    echo "Usage: $0 {move|remove} [x_m y_m]" >&2
    exit 2
    ;;
esac
