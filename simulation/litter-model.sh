#!/bin/bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/create3_ws/install/setup.bash
set -u

action="${1:-}"
kind="${2:-can}"
x="${3:-1.5}"
y="${4:-0.0}"
world="${SIM_WORLD:-room_camera}"
case "$kind" in
  can) model_file=/opt/pai/models/demo_litter_baseline/model.sdf ;;
  paper) model_file=/opt/pai/models/demo_litter_paper/model.sdf ;;
  bottle) model_file=/opt/pai/models/demo_litter_bottle/model.sdf ;;
  *) echo "Unknown litter type: $kind (use can, paper, or bottle)" >&2; exit 2 ;;
esac

remove_model() {
  gz service -s "/world/${world}/remove" \
    --reqtype gz.msgs.Entity \
    --reptype gz.msgs.Boolean \
    --timeout 3000 \
    --req 'name: "demo_litter", type: MODEL' >/dev/null 2>&1 || true
}

spawn_model() {
  ros2 run ros_gz_sim create \
    -world "$world" -file "$model_file" -name demo_litter \
    -x "$x" -y "$y" -z 0.0
}

case "$action" in
  add)
    spawn_model
    ;;
  remove)
    remove_model
    echo "Removed demo_litter if it existed."
    ;;
  move)
    remove_model
    spawn_model
    ;;
  *)
    echo "Usage: $0 {add|remove|move} [can|paper|bottle] [x_m y_m]" >&2
    exit 2
    ;;
esac
