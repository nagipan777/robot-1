#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
compose=(docker --context colima-create3 compose -f compose.yaml)
case "${1:-help}" in
  up)
    if ! colima -p create3 status >/dev/null 2>&1; then
      colima start create3 --cpu 2 --memory 4 --disk 24 --vm-type vz \
        --mount none --activate=false --ssh-config=false
    fi
    "${compose[@]}" up -d --build
    echo 'Gazebo: http://localhost:6080/vnc.html?autoconnect=true&resize=scale'
    ;;
  down) "${compose[@]}" down ;;
  stop) "${compose[@]}" down; colima -p create3 stop ;;
  shell) "${compose[@]}" exec simulator bash ;;
  status) "${compose[@]}" ps ;;
  logs) "${compose[@]}" logs --tail 100 -f ;;
  teleop)
    "${compose[@]}" exec simulator bash -c \
      'source /opt/create3_ws/install/setup.bash; ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p use_sim_time:=true'
    ;;
  test-marker)
    "${compose[@]}" exec simulator bash -lc \
      'source /opt/create3_ws/install/setup.bash; ros2 topic pub -r 2 /litter/target_marker visualization_msgs/msg/Marker "{header: {frame_id: odom}, ns: known_floor_test, id: 99, type: 2, action: 0, pose: {position: {x: 1.5, y: 0.0, z: 0.05}, orientation: {w: 1.0}}, scale: {x: 0.2, y: 0.2, z: 0.2}, color: {r: 0.0, g: 1.0, b: 0.0, a: 1.0}}"'
    ;;
  *) echo "Usage: $0 {up|down|stop|shell|status|logs|teleop|test-marker}" ;;
esac
