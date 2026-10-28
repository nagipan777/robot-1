# Wheel odometry monitoring integration report (2026-10-29)

## Result and boundary

The observation/logging implementation is complete for mock and future
`/roomba/encoder_counts` input. It is opt-in, warning-only, and has no STOP,
Drive, TCP, UART, TF, fusion, or Navigation output. **Live Pico/Roomba encoder
operation is not verified and the live packet 43/44 producer remains absent.**

No Pico, Roomba, ARM, Drive, or physical bridge was contacted during this
work. `ALLOW_NONZERO_DRIVE=False`, the Pico 500 ms watchdog, independent
400 ms drive-session limit, BRC-disabled setting, ARM/ACK gates, and existing
STOP behavior were not modified.

## Existing-path audit

| Path | Status | Evidence |
|---|---|---|
| Pico OI packet 43/44 UART acquisition | **not connected** | `pico/main.py` only queries packets 35, 41 and 42; packet 43/44 decode exists only in host/test helpers |
| Pico encoder transfer to Mac | **not connected** | TCP protocol has READY, STOP/OI diagnostics, ARM and sequenced Drive ACK; no encoder response frame |
| Physical bridge encoder receive/publish | **not connected** | `RoombaTcpTransport` has no encoder request/decoder; no producer publishes `/roomba/encoder_counts` |
| Mock/replay encoder input | **connected and tested** | JSON `std_msgs/String` on `/roomba/encoder_counts` feeds the existing wheel odometry and new monitor |
| Observation odometry | **connected for opt-in mock/future input** | `/roomba/encoder_counts` -> `wheel_odometry_node` -> `/roomba/odom`; no TF and no Navigation connection |
| Command comparison | **implemented, input-dependent** | new monitor observes `/roomba/cmd_vel_unstamped`, `/roomba/arm_generation`, and `/roomba/encoder_counts` |
| Current real-camera Navigation pose | **ArUco candidate, not wheel odometry** | `vision_controller.real_aruco_dry_run.yaml` consumes `/robot/pose_candidate` in `map`; `/roomba/odom` is not subscribed |
| ArUco/wheel fusion | **implemented preview, disabled for this work** | existing fusion design remains opt-in; no publisher switch was made |

The root `navigation.py` is a pure distance/heading calculation over coordinates
provided by its caller. In the real-camera AI runtime used on 2026-10-28, the
ROS controller's current pose and yaw came from the ArUco candidate topic, not
from wheel odometry.

## Frames, units, and time

- OI packets 43 (left) and 44 (right) are unsigned 16-bit cumulative encoder
  counts. Wrap-safe signed differences are calculated in ticks.
- The existing provisional conversion uses 508.8 counts/revolution, 0.072 m
  wheel diameter, and 0.235 m wheel separation. These are not yet measured on
  this physical Roomba.
- Wheel odometry starts at local `(0,0,0)`, uses metres/radians, and publishes
  frame `roomba_odom` with child `roomba_base_link`.
- ArUco is a calibrated floor projection in `map`, uses metres/radians, and
  stamps poses with the camera capture timestamp carried in the detection
  payload. It is not numerically averaged with local wheel coordinates.
- Encoder JSON uses a source timestamp in seconds. The monitor separately logs
  host receipt time and calculated data age. Missing, invalid, future, and old
  input is never converted to zero speed or a normal stop.
- `geometry_msgs/Twist` has no source timestamp, so command receipt time is the
  only command time currently available. ARM observation records a request,
  not an accepted ACK; accepted ARM state is not yet exported as a ROS topic.

## Added monitoring path

```text
/roomba/cmd_vel_unstamped (Twist request)
/roomba/arm_generation    (ARM request only)
/roomba/encoder_counts    (future bridge or mock JSON)
              |
              v
wheel_motion_monitor_node (explicit monitor_enabled=true)
              |
              +-- configurable warnings
              +-- flushed CSV event log
              +-- no STOP/Drive output
```

The monitor records receipt/source timestamps, requested left/right mm/s,
encoder values and deltas, per-sample estimated distance/yaw, data age,
requested ARM generation, and local request state. Configurable thresholds are
in `simulation/wheel_motion_monitor.yaml`:

- command-without-motion grace: 0.25 s;
- STOP-motion grace: 0.20 s;
- measurement stale limit: 0.50 s;
- encoder motion threshold: 1 tick;
- command motion threshold: 1 mm/s;
- maximum accepted delta: 2000 ticks.

Warnings are only candidates. Encoder rotation is not proof of floor motion or
physical stop, and no warning is connected to automatic STOP.

## Mock verification

Run without Pico or Docker networking:

```bash
./simulation/sim.sh wheel-monitor-dryrun
```

Output is saved under `artifacts/wheel_motion_monitor/dryrun/`. The deterministic
run produced 22 events and passed:

- `summary.json` SHA256
  `94f346c42a2b2c05cf1ef542c0afcd2b6e0a5a039ee4b664f0db100fb7270d7c`;
- `wheel_monitor_events.csv` SHA256
  `0b08db08e1160c01a9b7b035536741b39c275b7d8aae736148f0bc700e3dd188`.

- normal straight encoder change and estimated distance;
- differential/in-place turn and estimated yaw;
- STOP-following change inside grace as `stop_coast_observed`;
- change after grace as `wheel_change_after_stop`;
- non-zero request with no encoder change;
- never-received/stale input;
- invalid timeout input without zero-motion inference;
- unsigned 16-bit wraparound;
- disconnect/reconnect with a large value change re-baselined without motion;
- excessive connected delta rejected as a jump by unit tests.

Focused monitor, odometry, localization, bridge and Pico safety tests:
**90/90 PASS**. Full repository regression: **363/363 PASS**. Python compile
and shell syntax checks passed.

## Why live acquisition was not added tonight

The physical bridge is the only allowed TCP owner, but the deployed Pico has a
single UART owner and its current diagnostic query can wait up to 1000 ms for a
response. Adding a synchronous packet 43/44 query during a drive session could
delay processing of an incoming STOP beyond the approved safety bounds.
Therefore no unverified opcode or second TCP reader was added.

A compatible live extension needs a versioned encoder-response frame and a
cooperative/bounded Pico UART state machine in which STOP, session deadline,
and 500 ms watchdog checks pre-empt encoder work. Old hosts must ignore the new
capability and old Pico firmware must continue to work with the current bridge.
That protocol requires separate mock timing tests and a stationary real-UART
test before enabling the ROS producer.

## Still unverified / next attended test

1. With `ALLOW_NONZERO_DRIVE=False` and Roomba stationary, validate repeated
   packet 43/44 reads, left/right order, unsigned values, sample timestamps,
   timeout recovery, and no STOP-ACK latency regression.
2. Measure loaded wheel diameter, counts/revolution behavior, effective track,
   and left/right signs on this Roomba. Keep Navigation on ArUco.
3. In a separately approved single bounded trial, record command, Drive ACK,
   encoder samples, STOP, post-STOP samples, and human physical-stop evidence
   in one timeline. Tune warning thresholds from that evidence only.
4. Do not connect warnings to automatic STOP or switch Navigation/fusion until
   live acquisition, priority, timing, and failure tests pass.

Monitoring implementation PASS and physical encoder operation are deliberately
separate claims.
