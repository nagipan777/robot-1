# Roomba odometry dry-run and ArUco A/B report (2026-10-27)

## Result

The observation-only dry-run path is complete. It does not contact Pico or
Roomba, send ARM/STOP/Drive, publish TF, alter `/robot/pose`, or connect
odometry to Navigation. The feature remains opt-in (`odometry_enabled: false`).

Run the complete isolated check from the repository root:

```bash
./simulation/sim.sh odometry-dryrun-e2e
```

The command builds `Dockerfile.odometry-dryrun`, starts the container with
`--network none`, connects a CSV replay publisher to the real ROS 2 subscriber
and `/roomba/odom` publisher, saves CSV, then performs the synthetic ArUco A/B
comparison. Results are copied to `artifacts/odometry_dryrun/e2e/`.

## Architecture and safety boundary

```text
CSV replay (synthetic)
  -> /roomba/encoder_counts (std_msgs/msg/String JSON)
  -> wheel_odometry_node (explicit opt-in)
  -> /roomba/odom (nav_msgs/msg/Odometry)
  -> CSV and offline ArUco A/B comparison
```

`frame_id=roomba_odom`, `child_frame_id=roomba_base_link`; no TF is emitted.
ArUco `/robot/pose` is comparison-only. Its initial map-frame SE(2) pose is
removed before comparison, so map coordinates are never treated as odometry.

Changed scope: calculation, replay, observation node, configuration, RViz,
tests and documentation only. Unchanged/prohibited scope: Pico firmware,
UART ownership, TCP/ARM/STOP protocol, Drive, Navigation and Gazebo `/odom`.

## Input and state specification

CSV columns are:

`timestamp,left_encoder,right_encoder,valid,communication_state,sequence,error`

- Timestamp: finite, seconds, strictly increasing.
- Encoders: unsigned 16-bit OI packet 43/44 counts when connected and valid.
- `communication_state`: `connected`, `timeout`, or `disconnected`.
- Sequence: strictly increasing integer.
- Invalid/disconnected input is not integrated. Recovery first re-baselines.
- A gap greater than `max_time_gap_s` also re-baselines.
- Encoder jumps above `max_tick_delta` are rejected.

Output CSV includes timestamp, both raw counts and deltas, validity/state/error,
integration state, x/y/yaw in metres/radians, linear/angular velocity, relative
ArUco pose, position/yaw error and ArUco age. ROS covariance diagonals are
explicitly configured and validated.

Replay supports immediate execution (`replay_rate=0`) and scaled real-time
execution (`replay_rate>0`). Fixed inputs live in
`docs/odometry_dryrun_inputs/`: forward, reverse, left/right turn, stopped,
wraparound, disconnect/reconnect, abnormal jump, and a theoretical 26.8 degree
turn. The last is synthetic, not an actual Roomba measurement.

## ArUco A/B method

Both trajectories use metres, radians and seconds. The first ArUco pose is
removed as an SE(2) map origin. ArUco x/y are linearly interpolated at odometry
timestamps; yaw uses shortest-angle interpolation. Samples outside the maximum
time tolerance are counted unmatched. Output includes x/y/position/yaw errors,
RMSE, maxima, terminal errors, matched/unmatched counts, and mean/max absolute
time offset. `data_kind=synthetic` and `real_uart_used=false` prevent simulated
evidence from being reported as hardware evidence.

## Phase results

| Phase | Result | Evidence |
|---|---|---|
| P0 existing paths and boundary | PASS | Existing odometry, Navigation, ArUco, TCP and safety tests audited; safety paths unchanged |
| P1 calculation core/schema | PASS | deterministic differential-drive, wrap, delta, velocity, timestamp and jump tests |
| P2 CSV/replay/scenarios | PASS | fixed scenario files, validated reader, rate-selectable replay node |
| P3 ROS 2 integration | PASS | `/roomba/odom`, message/frame/stamp/twist/covariance and subscriber count verified |
| P4 faults/recovery | PASS | timeout does not integrate; reconnect re-baselines; jump/order/duplicate/gap tested |
| P5 CSV/RViz | PASS | ROS CSV generated; `roomba_odometry.rviz` displays `/roomba/odom` without TF display |
| P6 ArUco input | PASS | CSV reader, relative SE(2), interpolation and wrap-safe yaw tested |
| P6 A/B comparison | PASS (synthetic only) | 5 matched, 0 unmatched; position RMSE `1.34e-16 m`, yaw RMSE `7.62e-18 rad` |
| P7 isolated ROS E2E | PASS | 6 inputs, 5 odometry messages, one synthetic timeout correctly suppressed |

The ROS E2E final pose was x=`0.0142259145 m`, y=`0.0000538245 m`,
yaw=`0.0075670638 rad`. The run metadata records `tf_published=false`,
`navigation_connected=false`, `real_uart_used=false`, and
`nonzero_drive_sent=false`.

## Regression

- Focused dry-run/configuration suite: **29/29 PASS**.
- Full repository regression: **338/338 PASS** in the project virtualenv.
- First sandboxed full run: 314 passed and 24 loopback-TCP tests were blocked
  by local socket permissions; rerunning with loopback permission produced the
  338/338 result.
- Complete log: `artifacts/odometry_dryrun/e2e/unittest-338.log`.

## Still unverified on hardware

- OI packet 43/44 acquisition through the single Pico UART owner.
- Encoder sign and scale on this physical Roomba 680.
- Real wheel-lift trajectory and the prior approximately 26.8 degree result.
- Real ArUco-versus-encoder timing, drift, covariance and error metrics.

These items do not block the dry-run result. They do block claiming live
odometry or feeding it to Navigation. A future hardware producer must retain
STOP priority, bounded UART access, current ARM/TCP behavior and all drive
locks; this change deliberately does not add that producer.

## First user operation

With Roomba left disconnected, run exactly:

```bash
./simulation/sim.sh odometry-dryrun-e2e
```

Success is the final `PASS: odometry dry-run E2E artifacts saved ...` line.
