# Roomba 680 observation-only odometry prototype

## Scope and status

This prototype reuses `simulation/localization_core.py::WheelOdometry` and
publishes `nav_msgs/msg/Odometry` on `/roomba/odom`. It is opt-in, publishes no
TF, and does not publish or replace `/robot/pose`. Navigation therefore remains
on the existing ArUco-derived pose path.

The root `odometry.py` and its tests were audited first. `WheelOdometry` is the
existing ROS-independent hardened form of the same differential-drive
integration: it retains the 508.8 counts/revolution, 72 mm wheel diameter,
235 mm wheel separation, midpoint-heading integration, and 16-bit wrap logic,
and additionally enforces timestamps, jump rejection, and velocity output.

The ROS observation and mock-input path are implemented. Live OI packet 43/44
transport is deliberately **not added yet**: the current Pico has one UART and
one synchronous TCP session. A sensor query that waits for UART bytes can delay
STOP or desynchronize the following Drive ACK. Adding a request opcode without
solving that priority problem would violate the existing ARM/STOP safety
requirements.

## Data path

```text
mock/future serialized encoder producer
  -> /roomba/encoder_counts (std_msgs/String JSON)
  -> wheel-odometry-node (opt-in)
  -> /roomba/odom (nav_msgs/Odometry, roomba_odom frame)
  -> optional CSV only

/robot/pose (PoseStamped, map frame)
  -> relative-displacement comparison in CSV only
```

Input JSON has exactly the measured unsigned counts and acquisition timestamp:

```json
{"left":65010,"right":65010,"timestamp":1000.2}
```

Example `/roomba/odom` fields for that sample after a `65000,65000` baseline:

```yaml
header:
  stamp: {sec: 1000, nanosec: 200000000}
  frame_id: roomba_odom
child_frame_id: roomba_base_link
pose:
  pose:
    position: {x: 0.004446, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
twist:
  twist:
    linear: {x: 0.022228, y: 0.0, z: 0.0}
    angular: {x: 0.0, y: 0.0, z: 0.0}
```

Sample CSV: `docs/roomba_odometry_sample.csv`.

## Safety properties

- `odometry_enabled=false` by default.
- No TCP socket, UART access, ARM, STOP, or Drive code exists in the node.
- No TF is emitted.
- Encoder rollover uses the shortest signed 16-bit delta.
- Non-increasing timestamps, invalid counts, and jumps above
  `max_tick_delta` are rejected without updating the baseline.
- Pose starts at `(0, 0, 0)` on the first accepted encoder sample.
- ArUco map coordinates are never copied into odometry. The comparison removes
  the first ArUco SE(2) pose and compares relative motion only.
- CSV logging is disabled when `csv_path` is empty.

## Mock launch parameters

The container installs `/etc/roomba-odometry-observer.yaml`. To observe mock or
future serialized encoder samples, make a temporary config copy and explicitly
set `odometry_enabled: true`; optionally set `csv_path`. Do not enable this node
until an encoder producer exists. The node alone cannot contact Pico or Roomba.

## Live acquisition decision

OI Query List `149,2,43,44` returns four big-endian bytes: left packet 43 then
right packet 44. The old direct-serial scripts confirm that layout but bypass
the current TCP/ARM protocol and must not be used alongside the bridge.

The minimum acceptable future live design is one serialized UART owner on Pico
with explicit STOP priority, a bounded encoder query deadline, response framing,
and tests proving that an encoder timeout cannot delay STOP beyond the approved
safety bound or corrupt a subsequent Drive ACK. Until that is demonstrated,
the encoder producer remains absent and this feature stays mock/observation
only.

## Next attended validation

After the ARM/TCP gate and Roomba STOP `0x07`/wheels-zero/Safe-2 gates pass,
validate packet 43/44 while stationary first. Only then design the bounded
parallel acquisition trial for a separately approved wheel-lift, low-speed
run. A failure leaves `odometry_enabled=false` and the submission path
unchanged.

## Verification on 2026-10-27

- Python syntax checks: PASS.
- Odometry, localization, ARM/TCP, STOP, watchdog and driver regression:
  **156/156 PASS**.
- Docker/ROS runtime attempt: not run because the Colima Docker socket was not
  present. This was an environment availability failure before image build,
  not a node failure.
- Real Pico/Roomba encoder acquisition: not run and no protocol change was
  deployed.
