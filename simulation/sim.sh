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
    SIM_WORLD=room "${compose[@]}" up -d --build
    echo 'Gazebo: http://localhost:6080/vnc.html?autoconnect=true&resize=scale'
    ;;
  camera-up)
    if ! colima -p create3 status >/dev/null 2>&1; then
      colima start create3 --cpu 2 --memory 4 --disk 24 --vm-type vz \
        --mount none --activate=false --ssh-config=false
    fi
    SIM_WORLD=room_camera VISION_CONTROLLER_CONFIG=/etc/vision-controller.sim-camera.yaml \
      "${compose[@]}" up -d --build --force-recreate
    echo 'Camera world: http://localhost:6080/vnc.html?autoconnect=true&resize=scale'
    echo 'The fixed camera viewer opens automatically inside VNC.'
    echo 'MJPEG: http://127.0.0.1:8766/stream.mjpg'
    ;;
  nav-test-up)
    if colima -p create3 status >/dev/null 2>&1; then
      "${compose[@]}" down
      colima -p create3 stop
    fi
    colima start create3 --cpu 4 --memory 6 --disk 24 --vm-type vz \
      --mount none --activate=false --ssh-config=false
    SIM_WORLD=room VISION_CONTROLLER_CONFIG=/etc/vision-controller.sim-camera.yaml \
      ROBOT_X=0.0 ROBOT_Y=-0.75 ROBOT_YAW=0.0 SPAWN_DOCK=false \
      "${compose[@]}" up -d --build --force-recreate
    echo 'Known-target test world started with control disabled.'
    ;;
  down) "${compose[@]}" down ;;
  stop) "${compose[@]}" down; colima -p create3 stop ;;
  shell) "${compose[@]}" exec simulator bash ;;
  status) "${compose[@]}" ps ;;
  camera-status)
    "${compose[@]}" exec -T simulator bash -lc '
      set -e
      source /opt/ros/jazzy/setup.bash
      source /opt/create3_ws/install/setup.bash
      if [ "${SIM_WORLD:-room}" != "room_camera" ]; then
        echo "The camera world is not active. Run ./simulation/sim.sh camera-up" >&2
        exit 1
      fi
      echo "world=$SIM_WORLD"
      ros2 topic info /sim/fixed_camera/image_raw
      python3 -c '"'"'import urllib.request; print(urllib.request.urlopen("http://127.0.0.1:8766/health", timeout=2).read().decode())'"'"'
    '
    ;;
  litter-add|litter-move)
    if [ "$#" -eq 3 ]; then
      kind=can
      x="$2"
      y="$3"
    elif [ "$#" -eq 4 ]; then
      kind="$2"
      x="$3"
      y="$4"
    else
      echo "Usage: $0 ${1} [can|paper|bottle] X_M Y_M" >&2
      exit 2
    fi
    "${compose[@]}" exec -T simulator /usr/local/bin/litter-model "${1#litter-}" "$kind" "$x" "$y"
    ;;
  litter-remove)
    "${compose[@]}" exec -T simulator /usr/local/bin/litter-model remove can
    ;;
  calibration-marker-move)
    if [ "$#" -ne 3 ]; then
      echo "Usage: $0 calibration-marker-move X_M Y_M" >&2
      exit 2
    fi
    "${compose[@]}" exec -T simulator /usr/local/bin/calibration-marker move "$2" "$3"
    ;;
  calibration-marker-remove)
    "${compose[@]}" exec -T simulator /usr/local/bin/calibration-marker remove
    ;;
  logs) "${compose[@]}" logs --tail 100 -f ;;
  teleop)
    "${compose[@]}" exec simulator bash -c \
      'source /opt/create3_ws/install/setup.bash; ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p use_sim_time:=true'
    ;;
  test-marker)
    "${compose[@]}" exec simulator bash -lc \
      'source /opt/create3_ws/install/setup.bash; ros2 topic pub -r 2 /litter/target_marker visualization_msgs/msg/Marker "{header: {frame_id: odom}, ns: known_floor_test, id: 99, type: 2, action: 0, pose: {position: {x: 1.5, y: 0.0, z: 0.05}, orientation: {w: 1.0}}, scale: {x: 0.2, y: 0.2, z: 0.2}, color: {r: 0.0, g: 1.0, b: 0.0, a: 1.0}}"'
    ;;
  record-status)
    "${compose[@]}" exec simulator bash -lc \
      'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/recording-status'
    ;;
  presentation-layout)
    "${compose[@]}" exec -T simulator bash -lc '
      set -e
      export DISPLAY=:99
      for attempt in 1 2 3 4 5; do
        gazebo_id=$(wmctrl -l | awk "/Gazebo Sim/ {print \$1; exit}")
        rviz_id=$(wmctrl -l | awk "/RViz/ {print \$1; exit}")
        if [ -n "$gazebo_id" ] && [ -n "$rviz_id" ]; then
          wmctrl -i -r "$gazebo_id" -b remove,maximized_vert,maximized_horz
          wmctrl -i -r "$rviz_id" -b remove,maximized_vert,maximized_horz
          wmctrl -i -r "$gazebo_id" -e 0,0,0,760,760
          wmctrl -i -r "$rviz_id" -e 0,760,0,520,760
          wmctrl -i -a "$gazebo_id"
          exit 0
        fi
        sleep 2
      done
      echo "Gazebo or RViz window was not found" >&2
      exit 1
    '
    echo 'Gazebo and RViz arranged for a 1280x800 recording.'
    ;;
  ros-status)
    "${compose[@]}" exec simulator bash -lc '
      set -e
      source /opt/create3_ws/install/setup.bash
      retry_ros() {
        for attempt in 1 2 3; do
          "$@" && return 0
          [ "$attempt" -eq 3 ] || sleep 1
        done
        return 1
      }
      echo "=== ROS 2 nodes ==="
      ros2 node list
      echo "=== Required topics ==="
      ros2 topic info /webcam/detections
      ros2 topic info /litter/target
      ros2 topic info /robot/pose
      ros2 topic info /cmd_vel_unstamped
      echo "=== Controller parameters ==="
      retry_ros ros2 param get /vision_turn_controller calibration_ready
      retry_ros ros2 param get /vision_turn_controller control_mode
      retry_ros ros2 param get /vision_turn_controller control_enabled
    '
    ;;
  control-turn)
    "${compose[@]}" exec simulator bash -lc '
      set -e
      source /opt/create3_ws/install/setup.bash
      retry_ros() {
        for attempt in 1 2 3; do
          "$@" && return 0
          [ "$attempt" -eq 3 ] || sleep 1
        done
        return 1
      }
      retry_ros ros2 param set /vision_turn_controller control_enabled false
      retry_ros ros2 param set /vision_turn_controller calibration_ready true
      retry_ros ros2 param set /vision_turn_controller control_mode turn_only
      retry_ros ros2 param set /vision_turn_controller control_enabled true
    '
    echo 'Simulation control enabled in turn_only mode.'
    ;;
  control-approach)
    "${compose[@]}" exec simulator bash -lc '
      set -e
      source /opt/create3_ws/install/setup.bash
      retry_ros() {
        for attempt in 1 2 3; do
          "$@" && return 0
          [ "$attempt" -eq 3 ] || sleep 1
        done
        return 1
      }
      retry_ros ros2 param set /vision_turn_controller control_mode approach
    '
    echo 'Simulation control changed to approach mode.'
    ;;
  control-off)
    "${compose[@]}" exec simulator bash -lc '
      set -e
      source /opt/create3_ws/install/setup.bash
      retry_ros() {
        for attempt in 1 2 3; do
          "$@" && return 0
          [ "$attempt" -eq 3 ] || sleep 1
        done
        return 1
      }
      retry_ros ros2 param set /vision_turn_controller control_enabled false
      ros2 topic pub --once /cmd_vel_unstamped geometry_msgs/msg/Twist "{}"
    '
    echo 'Simulation control disabled and a zero velocity command was published.'
    ;;
  known-target-test|known-target-abort-test)
    if ! "${compose[@]}" exec -T simulator bash -lc '[ "$VISION_CONTROLLER_CONFIG" = /etc/vision-controller.sim-camera.yaml ]'; then
      echo 'Known-target tests require: ./simulation/sim.sh nav-test-up' >&2
      exit 1
    fi
    "${compose[@]}" exec -T simulator bash -lc '
      set -e
      source /opt/create3_ws/install/setup.bash
      odom_ready=false
      for attempt in {1..90}; do
        if ros2 topic type /odom 2>/dev/null | grep -q "nav_msgs/msg/Odometry"; then
          odom_ready=true
          break
        fi
        sleep 2
      done
      if [ "$odom_ready" != true ]; then
        echo "/odom did not become available; control remains disabled" >&2
        exit 1
      fi
      timeout 30s ros2 topic echo /odom --once >/dev/null
    '
    mkdir -p ../artifacts/known_target_test
    result_name=result.json
    if [ "$1" = known-target-abort-test ]; then
      result_name=abnormal_exit.json
    fi
    set +e
    if [ "$1" = known-target-abort-test ]; then
      "${compose[@]}" exec -T simulator bash -lc 'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/known-target-navigation-test --simulate-abort'
    else
      "${compose[@]}" exec -T simulator bash -lc 'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/known-target-navigation-test'
    fi
    test_status=$?
    set -e
    "${compose[@]}" cp simulator:/tmp/known_target_test_result.json "../artifacts/known_target_test/${result_name}"
    exit "$test_status"
    ;;
  oscillation-diagnostics)
    mkdir -p ../artifacts/oscillation_diagnostics
    "${compose[@]}" exec -T simulator bash -lc '
      source /opt/create3_ws/install/setup.bash
      exec /usr/local/bin/oscillation-diagnostics --duration "${1:-180}"
    ' -- "${2:-180}"
    "${compose[@]}" cp simulator:/tmp/oscillation_events.jsonl ../artifacts/oscillation_diagnostics/events.jsonl
    "${compose[@]}" cp simulator:/tmp/oscillation_summary.json ../artifacts/oscillation_diagnostics/summary.json
    ;;
  *)
    echo "Usage: $0 {up|camera-up|nav-test-up|down|stop|shell|status|camera-status|litter-add X Y|litter-move X Y|litter-remove|logs|teleop|test-marker|record-status|presentation-layout|ros-status|control-turn|control-approach|control-off|known-target-test|known-target-abort-test|oscillation-diagnostics [SECONDS]}"
    ;;
esac
