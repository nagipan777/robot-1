#!/bin/bash
set -e

if [ "${SIM_WORLD:-room}" != "room_camera" ]; then
    exit 0
fi

source /opt/ros/jazzy/setup.bash
source /opt/create3_ws/install/setup.bash
export DISPLAY=:99

for attempt in {1..90}; do
    if ros2 topic list 2>/dev/null | grep -qx '/sim/fixed_camera/image_raw'; then
        exec ros2 run rqt_image_view rqt_image_view /sim/fixed_camera/image_raw
    fi
    sleep 1
done

echo 'Fixed camera image topic did not become ready' >&2
exit 1
