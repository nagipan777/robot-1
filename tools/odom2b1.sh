#!/usr/bin/env bash
# Operator-facing wrapper for Phase 2B-1. Never issues ARM, Drive, or a motion command.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM="$ROOT/simulation/sim.sh"
PY="${PYTHON:-python3}"
INTERVAL="$ROOT/simulation/run_encoder_stop_interval.py"
BASE="$ROOT/artifacts/odometry_operator"
ACTIVE="$BASE/active-session.txt"

usage() {
  cat <<'EOF'
Roomba Phase 2B-1 — encoder measurement only (Mac terminal)

  ./tools/odom2b1.sh before PICO_IP
  ./tools/odom2b1.sh approve
  # Run separately approved bounded Drive using the existing test launcher.
  ./tools/odom2b1.sh after PICO_IP MEASURED_MM
  ./tools/odom2b1.sh status
  ./tools/odom2b1.sh new-session

This script never sends ARM or Drive. It does request STOP-first encoder diagnostics.
An existing simulator container and explicitly supervised, stationary Roomba are required.
Do not power-cycle the Roomba between the before/after readings.
EOF
}
active_dir() {
  if [[ ! -f "$ACTIVE" ]]; then
    echo "No active session. Run: ./tools/odom2b1.sh before PICO_IP" >&2
    exit 2
  fi
  local dir
  IFS= read -r dir < "$ACTIVE"
  if [[ -z "$dir" || ! -d "$dir" || "$dir" != "$BASE/"* ]]; then
    echo "Invalid session pointer: $ACTIVE" >&2
    exit 2
  fi
  printf '%s\n' "$dir"
}
get_state() {
  "$PY" - "$1" <<'PY'
import json,sys
with open(sys.argv[1], encoding="utf-8") as f:
    print(json.load(f).get("state", "unknown"))
PY
}
confirm() {
  local expected="$1" answer
  if [[ ! -t 0 ]]; then
    echo "Interactive confirmation required; refusing non-interactive operation." >&2
    exit 2
  fi
  printf 'Type %s: ' "$expected"
  IFS= read -r answer
  if [[ "$answer" != "$expected" ]]; then
    echo "Confirmation mismatch; no operation performed." >&2
    exit 2
  fi
}
require_ip() {
  if [[ -z "${1:-}" || "${1:-}" == -* || "${1:-}" == *" "* ]]; then
    echo "Specify Pico IP (e.g., 192.168.0.32); no default is assumed." >&2
    exit 2
  fi
}
mkdir -p "$BASE"
case "${1:-help}" in
  before)
    [[ "$#" == 2 ]] || { usage; exit 2; }
    require_ip "$2"
    if [[ -f "$ACTIVE" ]]; then
      echo "An active session exists. Use status or new-session (archives nothing)." >&2
      exit 2
    fi
    session="$(mktemp -d "$BASE/session-XXXXXXXX")"
    # A failed acquisition does not establish an active experiment.
    if ! "$SIM" roomba-encoder-ros-static "$2" STATIC_STOP_FIRST "$session/before" 10; then
      echo "Before acquisition failed. Incomplete evidence: $session" >&2
      exit 1
    fi
    "$PY" "$INTERVAL" prepare --before-csv "$session/before/encoder_status.csv" \
      --session "$session/interval.json" --motion-label "phase2b1_attended_straight"
    printf '%s\n' "$session" > "$ACTIVE"
    printf '\nBEFORE complete: %s\n' "$session"
    echo "Next: obtain separate approval for Drive, then ./tools/odom2b1.sh approve"
    ;;
  approve)
    [[ "$#" == 1 ]] || { usage; exit 2; }
    session="$(active_dir)"
    [[ "$(get_state "$session/interval.json")" == "before_acquired" ]] || {
      echo "Session is not awaiting approval." >&2; exit 2;
    }
    echo "This records the operator's decision ONLY. It does not ARM or Drive."
    echo "Confirm approval for one separately supervised bounded Drive trial."
    confirm ATTENDED_DRIVE_APPROVED
    "$PY" "$INTERVAL" authorize --session "$session/interval.json" \
      --confirmation ATTENDED_DRIVE_APPROVED
    echo "Approval recorded. Run the existing trial launcher separately; observe physical STOP."
    ;;
  after)
    [[ "$#" == 3 ]] || { usage; exit 2; }
    require_ip "$2"
    session="$(active_dir)"
    [[ "$(get_state "$session/interval.json")" == "drive_approved" ]] || {
      echo "Approval not recorded; run approve first." >&2; exit 2;
    }
    mm="$3"
    metres="$("$PY" - "$mm" <<'PY'
import math, sys
try:
    value=float(sys.argv[1])
except ValueError:
    raise SystemExit("MEASURED_MM must be a finite number")
if not math.isfinite(value):
    raise SystemExit("MEASURED_MM must be finite")
print(value/1000)
PY
)"
    echo "Only proceed after physically checking that the Roomba is stopped,"
    echo "has not restarted, and has not been power-cycled since BEFORE."
    confirm PHYSICAL_STOP_CONFIRMED
    "$SIM" roomba-encoder-ros-static "$2" STATIC_STOP_FIRST "$session/after" 10
    "$PY" "$INTERVAL" finalize --session "$session/interval.json" \
      --after-csv "$session/after/encoder_status.csv" \
      --output-csv "$session/result.csv" --summary "$session/summary.json" \
      --confirmation PHYSICAL_STOP_CONFIRMED --measured-distance-m "$metres" \
      --expected-left-sign 1 --expected-right-sign 1
    echo "Result: $session/result.csv"
    echo "Note: timestamps and session breaks may invalidate motion attribution."
    ;;
  status)
    [[ "$#" == 1 ]] || { usage; exit 2; }
    session="$(active_dir)"
    echo "Active session: $session"
    echo "Phase: $(get_state "$session/interval.json")"
    for f in "$session/before/encoder_status.csv" "$session/after/encoder_status.csv" "$session/result.csv"; do
      [[ ! -f "$f" ]] || echo "Saved: $f"
    done
    echo "This is FILE status, not bridge/ARM/Pico/live physical status."
    ;;
  new-session)
    [[ "$#" == 1 ]] || { usage; exit 2; }
    echo "This only clears the active pointer; it does not send STOP or change locks."
    confirm NEW_SESSION
    rm -f "$ACTIVE"
    echo "Old session directories and evidence retained."
    ;;
  help|-h|--help) usage ;;
  *) usage; exit 2 ;;
esac
