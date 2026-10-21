#!/bin/bash
set -e

if [ "${SIM_WORLD:-room}" != "room_camera" ]; then
    exit 0
fi

source /opt/ros/jazzy/setup.bash
source /opt/create3_ws/install/setup.bash

exec ros2 run ros_gz_bridge parameter_bridge \
    '/sim/fixed_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image' \
    --ros-args -r __node:=fixed_camera_image_bridge
