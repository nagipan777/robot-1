#!/bin/bash
set -e

if [ "${SIM_WORLD:-room}" != "room_camera" ]; then
  exit 0
fi

source /opt/ros/jazzy/setup.bash
source /opt/create3_ws/install/setup.bash
exec /usr/local/bin/sim-camera-mjpeg
