# Roomba 680 real encoder Phase 2A report (2026-10-29)

## Scope and result

Phase 2A adds an opt-in, STOP-first ROS 2 observation path around the Phase 1
encoder diagnostic.  This report covers source inspection, implementation,
unit/regression tests, a mock ROS 2 end-to-end test, and an attended real
static ROS 2 acquisition.  The real trial used no ARM, Drive, Navigation, or
TF and the wheels remained physically stopped.

Baseline source commit: `7c45386` on `fix-arm-ack-session-recovery`.

## Data path

```text
exclusive diagnostic TCP owner
  -> RoombaTcpTransport.connect() (initial STOP)
  -> STOP diagnostic (0x07, requested right=0, left=0)
  -> OI mode diagnostic (Safe=2)
  -> D7 07 encoder request
  -> D7 87 03 left_u16 right_u16 response
  -> /roomba/encoder_counts  UInt16MultiArray [left, right] (valid only)
  -> /roomba/encoder_status  JSON String (timestamp, validity, state, error)
  -> wheel_odometry_node
  -> /roomba/odom            nav_msgs/msg/Odometry
```

The static launcher shares `/tmp/roomba-physical-bridge.lock` with the physical
bridge and rejects a running/orphan physical owner.  It therefore does not
open a second Pico connection beside the bridge.  The Phase 1 wire protocol
and Pico UART implementation were not changed.

## Safety and isolation

- The encoder node is disabled by default and additionally requires the exact
  `STATIC_STOP_FIRST` confirmation.
- Connection setup verifies STOP ACK `0x07`, requested right/left `0/0`, and
  OI Safe mode `2` before encoder publication.
- The node has no ARM or Drive call.  The mock E2E records both as false.
- Encoder-query errors publish invalid status metadata but never republish an
  old count as a new measurement.
- `/roomba/odom` is observation-only.  TF is not broadcast and Navigation is
  not connected or reconfigured.
- The existing Pico 400 ms drive-session limit, 500 ms watchdog, STOP latch,
  BRC setting, and `ALLOW_NONZERO_DRIVE=False` were not modified.
- Phase 1's STOP-first query is not used as a running-drive poller.

## Odometry behavior

The existing `WheelOdometry` and dry-run reconnection state machine are reused.
The implementation:

- treats the first valid count as a baseline without integration;
- calculates signed 16-bit deltas across wraparound;
- does not integrate invalid/disconnected input;
- discards the baseline after communication loss or excessive time gap;
- rejects reversed/non-increasing timestamps and excessive tick jumps;
- rejects configurable excessive linear or angular speed before committing a
  pose update.

The retained geometry defaults are 508.8 counts/revolution, 0.072 m wheel
diameter, and 0.235 m wheel separation.  These are legacy project settings,
not a Phase 2A real calibration.  Wheel signs and real motion accuracy remain
unverified.

## Verification

### Host unit and regression tests

`python -m unittest discover -s tests -t .`: **392/392 PASS**.

This includes the Phase 1 regression baseline (386 tests) plus new Phase 2A
coverage for ROS source/installation, speed rejection, configuration, and
re-baselining behavior.  Covered behaviors include wraparound, initial
baseline, invalid input, communication loss/reconnect, reversed timestamps,
abnormal jumps, STOP-first constraints, and absence of ARM/Drive in the new
node.

### Mock ROS 2 E2E

Result: **PASS**.  Artifacts are under
`artifacts/real_encoder_phase2a/mock_ros2/`.

- counts: `(65534,100) -> (2,104) -> (6,108)`;
- wraparound deltas: `(+4,+4)` and `(+4,+4)`;
- initial sample: baseline only;
- `/roomba/odom`: mock updates produced from later valid samples;
- injected partial response `3/7`: one invalid status, zero invalid count
  publications;
- ARM sent: false; Drive sent: false; TF: false; Navigation connected: false.

Artifact SHA-256:

- `summary.json`: `53bba02a007632ffe766f909b7f55c7ba707994c68b139f2a6150cdc3e1b69ac`
- `encoder_status.csv`: `60b44dba6cfcba1f06e97fd689174e99ab9ea70aa30942889193bf4c81a5bb5a`
- `invalid_status.csv`: `8124cd0cb2315a7975b5ea26c0e562236a3f7700d101fad6efca3b123af15f9b`
- `odometry.csv`: `e3d42f4ded59e782e169228d42316d9073f974dc70eda727eb0e53fafe9f0a54`

## Real static verification

Status: **PASS for static diagnostics only** on 2026-10-29.

The first attended run acquired ten valid samples (`left=259`, `right=3960`)
with zero deltas.  A 1.236 s input gap caused the existing state machine to
re-baseline correctly, but review found that a baseline update was still being
published as a zero-velocity Odometry message.  That could misrepresent an
unintegrated sample as normal stopped motion.  The node was changed to suppress
`/roomba/odom` whenever `processed.integrated` is false.  The mock ROS 2 E2E and
all 392 regression tests passed again before the attended recheck.

The fixed attended run then acquired ten valid samples:

- left count: `263` for all ten samples;
- right count: `3964` for all ten samples;
- first sample: baseline, `integrated=False`, Odometry publish suppressed;
- samples 2--10: deltas `0/0`, position `0/0/0`, linear/angular velocity `0/0`;
- timeout, partial response, and invalid status: none;
- ARM and Drive: not sent;
- TCP owner after completion: zero;
- physical observation after completion: both wheels stopped, no abnormal
  sound, and no restart.

Artifacts:

- `artifacts/real_encoder_phase2a/20261029_static_ros2_fixed/encoder_status.csv`
  SHA-256 `49e143c738c8924055227e4d94677a995844b73cbaace983c380bebe8d4e3dab`
- `artifacts/real_encoder_phase2a/20261029_static_ros2_fixed/odometry.csv`
  SHA-256 `d5fe93ec53b704e0ec842bfb13e4a505e5ede1cc0360573deb8f11108bfa7504`

This result establishes real static count delivery and static ROS 2 odometry
behavior.  It does not establish wheel sign, moving accuracy, or floor pose.

## Reproduction command

After Pico/Roomba are attended, the deployed Pico source and all physical
safety conditions must be rechecked.  The prepared command is:

```bash
cd /Users/imaichika/robot/simulation
./sim.sh roomba-encoder-ros-static PICO_IP STATIC_STOP_FIRST OUTPUT_DIR 10 9000
```

This command requires attended approval and all physical gates.  Expected
acceptance evidence is 10 valid static count messages, zero deltas after
baseline, no Odometry publication for a baseline/unintegrated sample, CSV
output, and no wheel motion.  A
timeout, partial response, non-`0x03` encoder status, missing STOP/Safe gate,
or physical motion is a FAIL and must not be represented as a zero-speed
normal measurement.

## Known limitations

- Real Phase 2A static ROS topic delivery is verified for the attended fixed
  run described above; dynamic delivery is unverified.
- Encoder signs, metres-per-tick accuracy, and dynamic odometry accuracy are
  not established without a separately approved motion test.
- ROS `UInt16MultiArray` carries counts only.  Validity, acquisition time,
  connection state, and errors are deliberately carried on the paired status
  topic and in CSV.
- No sensor fusion, controller feedback, TF, or Navigation odometry input is
  part of Phase 2A.
