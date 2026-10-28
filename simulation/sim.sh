#!/bin/bash
set -euo pipefail
if [ "$(uname -m)" = arm64 ] && [ -x /opt/homebrew/bin/colima ]; then
  colima() {
    PATH="/opt/homebrew/bin:/opt/homebrew/sbin:$PATH" \
      /opt/homebrew/bin/colima "$@"
  }
fi
cd "$(dirname "$0")"
compose=(docker --context colima-create3 compose -f compose.yaml)
minimal_aruco_compose=(
  docker --context colima-create3 compose
  -f compose.yaml
  -f compose.real_camera_aruco_minimal.yaml
)
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
  vision-integration-up)
    if colima -p create3 status >/dev/null 2>&1; then
      "${compose[@]}" down
      colima -p create3 stop
    fi
    colima start create3 --cpu 4 --memory 6 --disk 24 --vm-type vz \
      --mount none --activate=false --ssh-config=false
    SIM_WORLD=room_camera \
      VISION_CONTROLLER_CONFIG=/etc/vision-controller.integration.yaml \
      ROBOT_X=0.0 ROBOT_Y=0.0 ROBOT_YAW=0.0 SPAWN_DOCK=false \
      "${compose[@]}" up -d --build --force-recreate
    for attempt in {1..15}; do
      if "${compose[@]}" exec -T simulator bash -lc \
        'source /opt/ros/jazzy/setup.bash; gz service -s /world/room_camera/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 2000 --req "pause: false"' \
        >/dev/null 2>&1; then
        break
      fi
      sleep 2
    done
    echo 'Vision integration world started with control and calibration disabled.'
    echo 'No real-Roomba bridge is started by this command.'
    ;;
  real-camera-dry-run-up)
    if [ "$#" -gt 5 ]; then
      echo "Usage: $0 real-camera-dry-run-up [PICO_IP [X_METRES Y_METRES YAW_RADIANS]]" >&2
      exit 2
    fi
    pico_host="${2:-}"
    dry_x="${3:-1.50}"
    dry_y="${4:-0.00}"
    dry_yaw="${5:-0.00}"
    if ! colima -p create3 status >/dev/null 2>&1; then
      colima start create3 --cpu 4 --memory 6 --disk 24 --vm-type vz \
        --mount none --activate=false --ssh-config=false
    fi
    SIM_WORLD=room VISION_CONTROLLER_CONFIG=/etc/vision-controller.real-dry-run.yaml \
      PICO_DRY_RUN_HOST="$pico_host" PICO_DRY_RUN_PORT=9001 \
      DRY_RUN_ROBOT_X="$dry_x" DRY_RUN_ROBOT_Y="$dry_y" DRY_RUN_ROBOT_YAW="$dry_yaw" \
      SPAWN_DOCK=false "${compose[@]}" up -d --build --force-recreate
    echo 'Real-camera dry-run started: control_enabled=false, calibration_ready=false.'
    echo 'Only /nav/cmd_vel_unstamped may carry calculated non-zero commands.'
    echo 'Publish a fresh map-frame pose on /robot/pose_dry_run before testing.'
    if [ -n "$pico_host" ]; then
      echo "Pico log-only bridge enabled at ${pico_host}:9001."
    fi
    ;;
  real-camera-aruco-dry-run-up)
    if [ "$#" -ne 1 ]; then
      echo "Usage: $0 real-camera-aruco-dry-run-up" >&2
      exit 2
    fi
    if ! colima -p create3 status >/dev/null 2>&1; then
      colima start create3 --cpu 4 --memory 6 --disk 24 --vm-type vz \
        --mount none --activate=false --ssh-config=false
    fi
    SIM_WORLD=room \
      VISION_CONTROLLER_CONFIG=/etc/vision-controller.real-aruco-dry-run.yaml \
      PICO_DRY_RUN_HOST="" SPAWN_DOCK=false \
      "${compose[@]}" up -d --build --force-recreate
    "${compose[@]}" exec -T simulator supervisorctl start aruco-pose >/dev/null
    echo 'Live-ArUco real-camera dry-run started.'
    echo 'Physical output remains disabled; only /roomba/cmd_vel_dry_run is published.'
    echo 'TCP 9000/9001 and the fixed dry-run pose publisher are not used.'
    ;;
  real-camera-aruco-minimal-up)
    if [ "$#" -ne 1 ]; then
      echo "Usage: $0 real-camera-aruco-minimal-up" >&2
      exit 2
    fi
    if ! colima -p create3 status >/dev/null 2>&1; then
      colima start create3 --cpu 2 --memory 4 --disk 24 --vm-type vz \
        --mount none --activate=false --ssh-config=false
    fi
    "${minimal_aruco_compose[@]}" up -d --build --force-recreate
    echo 'Minimal real-camera ArUco bridge started.'
    echo 'Only webcam-bridge and aruco-pose are configured; Create3/Gazebo/control/Pico are absent.'
    echo 'Mac producer command:'
    echo '.venv/bin/python vision.py --camera 0 --width 800 --height 600 --weights runs/detect/real_finetune/weights/best.pt --confidence 0.50 --imgsz 640 --aruco-marker-id 0 --aruco-dictionary DICT_4X4_50'
    ;;
  real-camera-aruco-minimal-status)
    "${minimal_aruco_compose[@]}" exec -T simulator \
      supervisorctl -c /etc/supervisor/real-camera-aruco-minimal.conf status
    ;;
  real-camera-aruco-minimal-down)
    "${minimal_aruco_compose[@]}" down
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
    for attempt in {1..15}; do
      if "${compose[@]}" exec -T simulator bash -lc \
        'gz service -s /world/room/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 2000 --req "pause: false"' \
        >/dev/null 2>&1; then
        break
      fi
      sleep 2
    done
    echo 'Known-target test world started with control disabled.'
    ;;
  down) "${compose[@]}" down ;;
  stop) "${compose[@]}" down; colima -p create3 stop ;;
  shell) "${compose[@]}" exec simulator bash ;;
  status) "${compose[@]}" ps ;;
  cmd-adapter)
    mode="${2:-dry_run}"
    input_topic="${3:-/cmd_vel_unstamped}"
    case "$mode" in
      dry_run|roomba) ;;
      gazebo)
        if [ "$input_topic" = /cmd_vel_unstamped ]; then
          echo "gazebo mode requires a distinct input such as /nav/cmd_vel_unstamped" >&2
          exit 2
        fi
        ;;
      *) echo "Usage: $0 cmd-adapter [dry_run|gazebo|roomba] [INPUT_TOPIC]" >&2; exit 2 ;;
    esac
    if [ "$mode" = dry_run ] && [ "$input_topic" = /nav/cmd_vel_unstamped ]; then
      "${compose[@]}" exec -T simulator supervisorctl restart cmd-vel-output-adapter >/dev/null 2>&1 || \
        "${compose[@]}" exec -T simulator supervisorctl start cmd-vel-output-adapter
      "${compose[@]}" exec -T simulator supervisorctl status cmd-vel-output-adapter
      exit 0
    fi
    "${compose[@]}" exec simulator bash -lc \
      'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/cmd-vel-output-adapter --ros-args --params-file /etc/cmd-vel-output-adapter.yaml -p mode:="$1" -p input_topic:="$2"' \
      _ "$mode" "$input_topic"
    ;;
  pico-cmd-dry-run)
    if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
      echo "Usage: $0 pico-cmd-dry-run PICO_IP [PORT]" >&2
      exit 2
    fi
    host="$2"
    port="${3:-9001}"
    "${compose[@]}" exec simulator bash -lc \
      'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/pico-dry-run-bridge --ros-args -p network_enabled:=true -p host:="$1" -p port:="$2" -p command_topic:=/roomba/cmd_vel_dry_run' \
      _ "$host" "$port"
    ;;
  e2e-record)
    if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
      echo "Usage: $0 e2e-record {A|B|C|D|E} [SECONDS]" >&2
      exit 2
    fi
    scenario="$2"
    seconds="${3:-15}"
    case "$scenario" in A|B|C|D|E) ;; *) echo "Scenario must be A, B, C, D, or E" >&2; exit 2 ;; esac
    container_output="/tmp/e2e-${scenario}.json"
    "${compose[@]}" exec -T simulator bash -lc \
      'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/e2e-dry-run-recorder "$1" --seconds "$2" --output "$3"' \
      _ "$scenario" "$seconds" "$container_output"
    artifact_dir="../artifacts/e2e_real_camera_pico_dry_run"
    mkdir -p "$artifact_dir"
    "${compose[@]}" cp "simulator:${container_output}" "$artifact_dir/result-${scenario}.json"
    "${compose[@]}" logs --no-color simulator > "$artifact_dir/ros2.log"
    echo "Saved $artifact_dir/result-${scenario}.json and ros2.log"
    ;;
  dry-run-pose)
    if [ "$#" -ne 4 ]; then
      echo "Usage: $0 dry-run-pose X_METRES Y_METRES YAW_RADIANS" >&2
      exit 2
    fi
    "${compose[@]}" exec simulator bash -lc \
      'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/dry-run-pose-publisher "$1" "$2" "$3"' \
      _ "$2" "$3" "$4"
    ;;
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
  litter-add|litter-move|litter-pose)
    if [ "$#" -eq 3 ]; then
      kind=can
      x="$2"
      y="$3"
      yaw=0.0
    elif [ "$#" -eq 4 ] || [ "$#" -eq 5 ]; then
      kind="$2"
      x="$3"
      y="$4"
      yaw="${5:-0.0}"
    else
      echo "Usage: $0 ${1} [can|paper|bottle] X_M Y_M [YAW_RAD]" >&2
      exit 2
    fi
    "${compose[@]}" exec -T simulator /usr/local/bin/litter-model "${1#litter-}" "$kind" "$x" "$y" "$yaw"
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
  roomba-tcp)
    if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
      echo "Usage: $0 roomba-tcp PICO_IP [PORT]" >&2
      exit 2
    fi
    host="$2"
    port="${3:-9000}"
    "${compose[@]}" exec simulator bash -lc \
      'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/roomba-bridge-tcp --ros-args -p host:="$1" -p port:="$2" -p drive_enabled:=false -p watchdog_test_mode:=false -p command_topic:=/roomba/cmd_vel_unstamped' \
      -- "$host" "$port"
    ;;
  roomba-tcp-diagnostic)
    if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
      echo "Usage: $0 roomba-tcp-diagnostic PICO_IP [PORT]" >&2
      exit 2
    fi
    host="$2"
    port="${3:-9000}"
    "${compose[@]}" exec simulator bash -lc \
      'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/roomba-bridge-tcp --ros-args -p host:="$1" -p port:="$2" -p drive_enabled:=false -p diagnostic_on_connect:=true -p watchdog_test_mode:=false -p command_topic:=/roomba/cmd_vel_unstamped' \
      -- "$host" "$port"
    ;;
  roomba-short-e-bridge)
    if [ "$#" -lt 3 ] || [ "$#" -gt 4 ] || [ "$3" != "WHEELS_LIFTED" ]; then
      echo "Usage: $0 roomba-short-e-bridge PICO_IP WHEELS_LIFTED [PORT]" >&2
      exit 2
    fi
    host="$2"
    port="${4:-9000}"
    short_e_owner="$$-$(date +%s)-${RANDOM}"
    short_e_pid_file=/tmp/roomba-short-e-bridge.pid

    cleanup_short_e_bridge() {
      # docker compose exec can outlive the host shell when Ctrl+C is used
      # through a pipe (for example, `... | tee bridge.log`).  Stop only the
      # bridge owned by this invocation; never terminate another trial.
      "${compose[@]}" exec -T simulator bash -lc '
        pid_file="$1"
        expected_owner="$2"
        [ -r "$pid_file" ] || exit 0
        read -r pid owner < "$pid_file" || exit 0
        [ "$owner" = "$expected_owner" ] || exit 0
        case "$pid" in ""|*[!0-9]*) exit 0 ;; esac
        if kill -0 "$pid" 2>/dev/null &&
           tr "\000" " " < "/proc/$pid/cmdline" | grep -q "/usr/local/bin/roomba-bridge-tcp"; then
          kill -INT "$pid" 2>/dev/null || true
          count=0
          while kill -0 "$pid" 2>/dev/null && [ "$count" -lt 50 ]; do
            sleep 0.1
            count=$((count + 1))
          done
          if kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null || true
          fi
        fi
        current_owner=""
        if [ -r "$pid_file" ]; then
          read -r _ current_owner < "$pid_file" || true
        fi
        if [ "$current_owner" = "$expected_owner" ]; then
          rm -f "$pid_file"
        fi
      ' -- "$short_e_pid_file" "$short_e_owner" >/dev/null 2>&1 || true
    }

    short_e_signal_status=0
    short_e_exec_pid=""
    short_e_launcher_pid=$$
    keep_short_e_stdin_open() {
      # Heartbeats form a lifecycle lease. If this host command is interrupted
      # (including when stdout is piped through tee), heartbeats stop even if
      # Docker leaves the exec stdin stream open.
      while kill -0 "$short_e_launcher_pid" 2>/dev/null; do
        printf '.\n' 2>/dev/null || return 0
        sleep 0.2
      done
    }
    interrupt_short_e_bridge() {
      short_e_signal_status="$1"
      if [ -n "$short_e_exec_pid" ] && kill -0 "$short_e_exec_pid" 2>/dev/null; then
        kill -INT "$short_e_exec_pid" 2>/dev/null || true
      fi
      cleanup_short_e_bridge
    }
    trap 'interrupt_short_e_bridge 130' INT
    trap 'interrupt_short_e_bridge 143' TERM
    trap cleanup_short_e_bridge EXIT
    set +e
    keep_short_e_stdin_open | "${compose[@]}" exec -T simulator bash -lc '
      source /opt/create3_ws/install/setup.bash
      pid_file="$1"
      owner="$2"
      host="$3"
      port="$4"
      exec 9>/tmp/roomba-short-e-bridge.lock
      if ! flock -n 9; then
        echo "Refusing duplicate roomba-short-e-bridge: another instance is running" >&2
        exit 73
      fi
      bridge_pid=""
      stdin_monitor_pid=""
      stop_bridge() {
        if [ -n "$bridge_pid" ] && kill -0 "$bridge_pid" 2>/dev/null; then
          kill -INT "$bridge_pid" 2>/dev/null || true
          wait "$bridge_pid" 2>/dev/null || true
        fi
        if [ -n "$stdin_monitor_pid" ]; then
          kill "$stdin_monitor_pid" 2>/dev/null || true
          wait "$stdin_monitor_pid" 2>/dev/null || true
        fi
        rm -f "$pid_file"
      }
      trap stop_bridge EXIT INT TERM
      /usr/local/bin/roomba-bridge-tcp --ros-args -p host:="$host" -p port:="$port" -p drive_enabled:=true -p diagnostic_on_connect:=true -p watchdog_test_mode:=false -p max_speed_mm_s:=20 -p command_timeout:=0.5 -p max_drive_session_duration:=0.4 -p command_topic:=/roomba/cmd_vel_unstamped &
      bridge_pid=$!
      printf "%s %s\n" "$bridge_pid" "$owner" > "$pid_file"
      exec 8<&0
      # Do not trust EOF alone: Docker may retain the exec stream after its
      # client is interrupted. A missing heartbeat expires this wrapper.
      (
        heartbeat_count=0
        heartbeat_timeouts=0
        heartbeat_status=0
        while true; do
          if IFS= read -r -t 0.5 heartbeat; then
            heartbeat_timeouts=0
          else
            heartbeat_status=$?
            # EOF means the host-side exec is gone and must stop immediately.
            # A single timed-read miss can occur under camera/AI load, so only
            # expire the lifecycle lease after two consecutive 0.5 s misses.
            if [ "$heartbeat_status" -eq 1 ]; then
              break
            fi
            heartbeat_timeouts=$((heartbeat_timeouts + 1))
            if [ "$heartbeat_timeouts" -ge 2 ]; then
              break
            fi
            continue
          fi
          [ "$heartbeat" = "." ] || continue
          heartbeat_count=$((heartbeat_count + 1))
        done <&8
        echo "short-E lifecycle heartbeat ended: read_status=$heartbeat_status received=$heartbeat_count" >&2
        exit "$heartbeat_status"
      ) &
      stdin_monitor_pid=$!
      set +e
      wait -n "$bridge_pid" "$stdin_monitor_pid"
      first_status=$?
      set -e
      if kill -0 "$bridge_pid" 2>/dev/null; then
        # Host-side docker exec disappeared: make Python run its normal
        # KeyboardInterrupt/finally STOP path before this wrapper exits.
        kill -INT "$bridge_pid" 2>/dev/null || true
        wait "$bridge_pid" 2>/dev/null || true
        exit 130
      fi
      exit "$first_status"
    ' -- "$short_e_pid_file" "$short_e_owner" "$host" "$port" &
    short_e_exec_pid=$!
    wait "$short_e_exec_pid"
    short_e_status=$?
    set -e
    cleanup_short_e_bridge
    trap - EXIT INT TERM
    if [ "$short_e_signal_status" -ne 0 ]; then
      exit "$short_e_signal_status"
    fi
    exit "$short_e_status"
    ;;
  roomba-tcp-watchdog)
    if [ "$#" -lt 3 ] || [ "$#" -gt 4 ] || [ "$3" != "WHEELS_LIFTED" ]; then
      echo "Usage: $0 roomba-tcp-watchdog PICO_IP WHEELS_LIFTED [PORT]" >&2
      exit 2
    fi
    host="$2"
    port="${4:-9000}"
    "${compose[@]}" exec simulator bash -lc \
      'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/roomba-bridge-tcp --ros-args -p host:="$1" -p port:="$2" -p drive_enabled:=true -p diagnostic_on_connect:=true -p watchdog_test_mode:=true -p watchdog_test_silence:=0.75 -p watchdog_test_confirmation:=WHEELS_LIFTED -p max_speed_mm_s:=20 -p command_topic:=/roomba/cmd_vel_unstamped' \
      -- "$host" "$port"
    ;;
  roomba-wheel-lift-test)
    mode="${2:-stop}"
    case "$mode" in
      stop|rejection)
        rotation_authorized=false
        ;;
      rotate|watchdog)
        if [ "${3:-}" != "WHEELS_LIFTED" ]; then
          echo "Refusing $mode: place Roomba on a stable stand and pass WHEELS_LIFTED" >&2
          exit 2
        fi
        rotation_authorized=true
        ;;
      *)
        echo "Usage: $0 roomba-wheel-lift-test {stop|rejection|rotate|watchdog} [WHEELS_LIFTED]" >&2
        exit 2
        ;;
    esac
    if [ "$rotation_authorized" = true ]; then
      arm_generation="$(date +%s)"
      "${compose[@]}" exec simulator bash -lc \
        'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/roomba-wheel-lift-test --ros-args -p mode:="$1" "${@:2}"' \
        -- "$mode" -p allow_rotation:=true -p confirmation:=WHEELS_LIFTED -p arm_generation:="$arm_generation"
    else
      "${compose[@]}" exec simulator bash -lc \
        'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/roomba-wheel-lift-test --ros-args -p mode:="$1"' \
        -- "$mode"
    fi
    ;;
  aruco-pose)
    "${compose[@]}" exec simulator bash -lc \
      'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/aruco-pose-publisher --ros-args --params-file /etc/aruco-pose.yaml'
    ;;
  aruco-pose-start)
    "${compose[@]}" exec -T simulator supervisorctl start aruco-pose >/dev/null
    "${compose[@]}" exec -T simulator supervisorctl status aruco-pose
    ;;
  aruco-pose-stop)
    "${compose[@]}" exec -T simulator supervisorctl stop aruco-pose >/dev/null
    "${compose[@]}" exec -T simulator supervisorctl status aruco-pose || true
    ;;
  localization-wheel-preview)
    "${compose[@]}" exec simulator bash -lc \
      'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/wheel-odometry-node --ros-args --params-file /etc/localization-preview.yaml'
    ;;
  localization-fusion-preview)
    "${compose[@]}" exec simulator bash -lc \
      'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/localization-fusion-node --ros-args --params-file /etc/localization-preview.yaml'
    ;;
  localization-preview-test)
    "${compose[@]}" exec -T simulator bash -lc \
      'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/localization-preview-test --ros-args --params-file /etc/localization-preview.yaml'
    ;;
  odometry-dryrun-e2e)
    if [ "$#" -ne 1 ]; then
      echo "Usage: $0 odometry-dryrun-e2e" >&2
      exit 2
    fi
    if ! colima -p create3 status >/dev/null 2>&1; then
      colima start create3 --cpu 2 --memory 4 --disk 24 --vm-type vz \
        --mount none --activate=false --ssh-config=false
    fi
    image=robot-odometry-dryrun
    artifact_dir="$(cd .. && pwd)/artifacts/odometry_dryrun/e2e"
    container="robot-odometry-dryrun-e2e-$$"
    mkdir -p "$artifact_dir"
    docker --context colima-create3 build \
      -f Dockerfile.odometry-dryrun -t "$image" .
    docker --context colima-create3 create --name "$container" --network none \
      "$image" bash -lc '
        set -euo pipefail
        set +u
        source /opt/ros/jazzy/setup.bash
        set -u
        mkdir -p /output
        odometry-ros-e2e \
          --input /etc/odometry-dryrun-e2e.csv \
          --output-csv /output/ros_odometry.csv \
          --output-json /output/ros_e2e_summary.json
        run-odometry-dryrun \
          --input /etc/odometry-dryrun-e2e.csv \
          --output-dir /output/offline
      '
    cleanup_odometry_dryrun() {
      docker --context colima-create3 rm -f "$container" >/dev/null 2>&1 || true
    }
    trap cleanup_odometry_dryrun EXIT INT TERM
    docker --context colima-create3 start -a "$container"
    docker --context colima-create3 cp "$container:/output/." "$artifact_dir"
    cleanup_odometry_dryrun
    trap - EXIT INT TERM
    echo "PASS: odometry dry-run E2E artifacts saved to $artifact_dir"
    ;;
  localization-hardware-live)
    echo 'Refusing hardware-live localization: UART/TCP 9000 is intentionally not implemented in this phase.' >&2
    exit 1
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
  replay-create3-trajectory)
    trajectory_file="${2:-../artifacts/submission/pai_create3_20261015T163837JST/result.json}"
    replay_speed="${3:-10}"
    if [ "$#" -gt 3 ] || [ ! -f "$trajectory_file" ]; then
      echo "Usage: $0 replay-create3-trajectory [RESULT_JSON [SPEED]]" >&2
      exit 2
    fi
    if ! "${compose[@]}" exec -T simulator bash -lc '[ "$SIM_WORLD" = room_camera ]'; then
      echo 'Trajectory replay requires: ./simulation/sim.sh vision-integration-up' >&2
      exit 1
    fi
    "${compose[@]}" cp "$trajectory_file" simulator:/tmp/create3-trajectory-result.json
    "${compose[@]}" cp replay_create3_trajectory.py simulator:/tmp/replay-create3-trajectory.py
    "${compose[@]}" exec -T simulator bash -lc '
      set -e
      source /opt/create3_ws/install/setup.bash
      if ros2 topic list | grep -qx /roomba/cmd_vel_unstamped; then
        echo "Refusing replay: physical Roomba command topic exists" >&2
        exit 1
      fi
      ros2 param set /vision_turn_controller control_enabled false >/dev/null
      ros2 param set /vision_turn_controller calibration_ready false >/dev/null
      ros2 topic pub --once /cmd_vel_unstamped geometry_msgs/msg/Twist "{}" >/dev/null
      visualizer_was_running=false
      if supervisorctl status presentation-visualizer | grep -q RUNNING; then
        supervisorctl stop presentation-visualizer >/dev/null
        visualizer_was_running=true
      fi
      cleanup_replay() {
        ros2 topic pub --once /cmd_vel_unstamped geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true
        if [ "$visualizer_was_running" = true ]; then
          supervisorctl start presentation-visualizer >/dev/null 2>&1 || true
        fi
      }
      trap cleanup_replay EXIT
      python3 /tmp/replay-create3-trajectory.py \
        /tmp/create3-trajectory-result.json --world room_camera --speed "$1" --fps 5
    ' -- "$replay_speed"
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
  multi-target-test)
    if ! "${compose[@]}" exec -T simulator bash -lc '[ "$VISION_CONTROLLER_CONFIG" = /etc/vision-controller.sim-camera.yaml ]'; then
      echo 'Multi-target test requires: ./simulation/sim.sh nav-test-up' >&2
      exit 1
    fi
    "${compose[@]}" exec -T simulator bash -lc '
      set -e
      source /opt/create3_ws/install/setup.bash
      supervisorctl stop webcam-bridge >/dev/null 2>&1 || true
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
      exec /usr/local/bin/multi-target-navigation-test
    '
    test_status=$?
    mkdir -p ../artifacts/multi_target_test
    "${compose[@]}" cp simulator:/tmp/multi_target_test_result.json ../artifacts/multi_target_test/result.json
    "${compose[@]}" cp simulator:/tmp/multi_target_test_report.md ../artifacts/multi_target_test/REPORT.md
    exit "$test_status"
    ;;
  synthetic-coordinate-test)
    if ! "${compose[@]}" exec -T simulator bash -lc \
      '[ "$SIM_WORLD" = room_camera ] && [ "$VISION_CONTROLLER_CONFIG" = /etc/vision-controller.integration.yaml ] && [ "$ROBOT_X" = 0.0 ] && [ "$ROBOT_Y" = 0.0 ] && [ "$ROBOT_YAW" = 0.0 ] && [ "$SPAWN_DOCK" = false ]'; then
      echo 'Coordinate test requires: ./simulation/sim.sh vision-integration-up' >&2
      exit 1
    fi
    set +e
    "${compose[@]}" exec -T simulator bash -lc \
      'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/synthetic-detection-coordinate-test'
    test_status=$?
    set -e
    mkdir -p ../artifacts/synthetic_detection_test
    "${compose[@]}" cp simulator:/tmp/synthetic_detection_test_result.json \
      ../artifacts/synthetic_detection_test/result.json
    exit "$test_status"
    ;;
  yolo-navigation-test)
    if ! "${compose[@]}" exec -T simulator bash -lc \
      '[ "$SIM_WORLD" = room_camera ] && [ "$VISION_CONTROLLER_CONFIG" = /etc/vision-controller.integration.yaml ] && [ "$SPAWN_DOCK" = false ]'; then
      echo 'YOLO navigation test requires: ./simulation/sim.sh vision-integration-up' >&2
      exit 1
    fi
    set +e
    "${compose[@]}" exec -T simulator bash -lc \
      'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/yolo-navigation-test'
    test_status=$?
    set -e
    mkdir -p ../artifacts/yolo_navigation_test
    "${compose[@]}" cp simulator:/tmp/yolo_navigation_test_result.json \
      ../artifacts/yolo_navigation_test/result.json
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
    echo "Usage: $0 {up|camera-up|vision-integration-up|real-camera-dry-run-up [PICO_IP [X Y YAW]]|real-camera-aruco-dry-run-up|real-camera-aruco-minimal-up|real-camera-aruco-minimal-status|real-camera-aruco-minimal-down|nav-test-up|down|stop|shell|status|cmd-adapter [dry_run|gazebo|roomba] [INPUT_TOPIC]|pico-cmd-dry-run PICO_IP [PORT]|e2e-record {A|B|C|D|E} [SECONDS]|dry-run-pose X Y YAW|camera-status|litter-add X Y|litter-move X Y|litter-remove|logs|teleop|roomba-tcp PICO_IP [PORT]|roomba-tcp-diagnostic PICO_IP [PORT]|roomba-short-e-bridge PICO_IP WHEELS_LIFTED [PORT]|roomba-tcp-watchdog PICO_IP WHEELS_LIFTED [PORT]|roomba-wheel-lift-test MODE [WHEELS_LIFTED]|aruco-pose|aruco-pose-start|aruco-pose-stop|localization-wheel-preview|localization-fusion-preview|localization-preview-test|localization-hardware-live|test-marker|record-status|presentation-layout|replay-create3-trajectory [RESULT_JSON [SPEED]]|ros-status|control-turn|control-approach|control-off|known-target-test|known-target-abort-test|multi-target-test|synthetic-coordinate-test|yolo-navigation-test|oscillation-diagnostics [SECONDS]}"
    ;;
esac
