#!/bin/bash
set -e

source /opt/create3_ws/install/setup.bash
config="${VISION_CONTROLLER_CONFIG:-/etc/vision-controller.yaml}"
if [ "${SIM_WORLD:-room}" = "room_camera" ] && [ -z "${VISION_CONTROLLER_CONFIG:-}" ]; then
  config=/etc/vision-controller.sim-camera.yaml
fi

exec /usr/local/bin/vision-turn-controller --ros-args --params-file "$config"
