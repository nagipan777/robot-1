#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/create3_ws/install/setup.bash
for attempt in {1..60}; do
    if xdpyinfo -display :99 >/dev/null 2>&1; then
        exec ros2 launch irobot_create_gz_bringup create3_gz.launch.py \
            world:=room use_rviz:=true
            
    fi
    sleep 1
done
echo 'X display did not become ready' >&2
exit 1
