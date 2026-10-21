#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/create3_ws/install/setup.bash
world="${SIM_WORLD:-room}"
for attempt in {1..60}; do
    if xdpyinfo -display :99 >/dev/null 2>&1; then
        exec ros2 launch irobot_create_gz_bringup create3_gz.launch.py \
            world:="$world" use_rviz:=true \
            x:="${ROBOT_X:-0.0}" y:="${ROBOT_Y:-0.0}" \
            yaw:="${ROBOT_YAW:-0.0}" spawn_dock:="${SPAWN_DOCK:-true}"
            
    fi
    sleep 1
done
echo 'X display did not become ready' >&2
exit 1
