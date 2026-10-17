#!/bin/bash
set -e

if [ "${SIM_WORLD:-room}" != "room_camera" ]; then
  exit 0
fi

source /opt/ros/jazzy/setup.bash
exec gz topic -e -t /sim/fixed_camera/image_raw >/dev/null
