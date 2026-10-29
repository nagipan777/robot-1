# Roomba 680 Phase 2B-1 STOP-to-STOP encoder plan (2026-10-29)

## Status

Software preparation and mock tests: **PASS**.  Real movement: **not run**.

Baseline is Phase 2A commit `44d3795`, including the attended static ROS 2
result in `docs/REAL_ENCODER_PHASE2A_REPORT_20261029.md`.  This phase does not
add periodic encoder reads during motion and does not connect odometry to TF,
Navigation, or control.

## Reused paths

- Before/after acquisition reuses `sim.sh roomba-encoder-ros-static`, including
  exclusive TCP ownership, initial STOP, STOP ACK `0x07`, requested wheel speeds
  `0/0`, Safe mode `2`, and the Phase 1 `D7 07` / `D7 87` query.
- Movement reuses the existing attended bridge and bounded one-shot Drive
  publisher.  Phase 2B-1 does not implement another ARM, Drive, TCP, or UART
  path.
- Delta calculation reuses `localization_core.encoder_delta`, equivalent to
  `((current - previous + 32768) % 65536) - 32768`.
- Existing geometry remains 508.8 ticks/revolution, 0.072 m wheel diameter,
  and 0.235 m separation.  These are configuration values, not a real-motion
  calibration claim.

## New offline harness

`simulation/run_encoder_stop_interval.py` is deliberately offline.  It has no
socket owner and no ARM or Drive method.

1. `prepare` validates a successful before-acquisition CSV and freezes its last
   unsigned 16-bit counts in a session JSON.
2. `authorize` records the exact `ATTENDED_DRIVE_APPROVED` token only after the
   separate human approval.  It does not send ARM or Drive.
3. `finalize` is rejected unless the session was approved and the exact
   `PHYSICAL_STOP_CONFIRMED` token accompanies a valid after-acquisition CSV.
4. The result CSV contains raw before/after counts, signed deltas, per-wheel
   estimated distance, estimated centre distance/yaw, optional measured
   distance/yaw, errors, expected signs, sign matches, and abnormal-jump flag.

STOP/Safe failure, encoder failure, timeout/partial data, or disconnected input
aborts rather than retries or converts old data into a measurement.

## Attended procedure (not yet authorized)

Every physical substep remains one-at-a-time and requires physical observation.

1. Recheck Pico source and `ALLOW_NONZERO_DRIVE=False`, BRC false, 400 ms
   session limit, and 500 ms watchdog.
2. Manually wake Roomba and confirm stationary wheels.
3. With individual static-diagnostic approval, acquire the before CSV using
   the Phase 2A STOP-first command.
4. Run `prepare` to freeze the before endpoint.
5. Check the route, independent stop, single bridge/publisher/subscriber, STOP
   ACK, `0/0`, Safe 2, and communication quality.
6. Obtain a new, trial-specific attended Drive approval.  Only then record the
   approval token and temporarily use the existing one-shot Drive path.
7. First trial limit: straight wheels, no more than 20 mm/s and 400 ms.  This is
   equal to the previously validated limit; it is not expanded in Phase 2B-1.
8. Confirm physical STOP and no restart.  Do not re-ARM or retry that trial.
9. Return `ALLOW_NONZERO_DRIVE=False`, restart safely, and recheck STOP/Safe.
10. With a separate static-diagnostic approval, acquire the after CSV.  No
    query is sent while Drive is active.
11. Run `finalize` with the physical-stop token and manually measured distance
    or yaw, then review sign and anomaly fields.

Offline commands after their corresponding evidence exists:

```bash
.venv/bin/python simulation/run_encoder_stop_interval.py prepare \
  --before-csv BEFORE/encoder_status.csv \
  --session SESSION.json --motion-label straight_20mm_s_400ms

.venv/bin/python simulation/run_encoder_stop_interval.py authorize \
  --session SESSION.json --confirmation ATTENDED_DRIVE_APPROVED

.venv/bin/python simulation/run_encoder_stop_interval.py finalize \
  --session SESSION.json --after-csv AFTER/encoder_status.csv \
  --output-csv interval.csv --summary summary.json \
  --confirmation PHYSICAL_STOP_CONFIRMED \
  --measured-distance-m 0.0 --expected-left-sign 1 --expected-right-sign 1
```

The measured value above is an example placeholder and must be replaced by an
actual human measurement; it must never be inferred from the encoder result.

## Verification

Focused Phase 2B-1 tests cover:

- same-direction ticks;
- opposite-direction ticks;
- unsigned 16-bit wraparound;
- STOP/Safe gate failure;
- encoder/connection failure with no retry;
- abnormal tick jump;
- unapproved Drive-state rejection;
- missing physical-stop confirmation rejection;
- invalid acquisition CSV rejection;
- offline CLI prepare/finalize output.

Result: **7/7 focused tests PASS; 399/399 full regression PASS** after the
source-safety assertion was added.  (The earlier pre-assertion run was 6/6 and
398/398.)

## Risks and unresolved items

- Left/right signs are intentionally not declared correct until the first
  attended movement result is compared with observed direction.
- Distance and yaw accuracy are unknown; configured geometry may require
  calibration.
- Roomba must not be power-cycled between before and after acquisitions or the
  interval is invalid.
- A large delta is reported as anomalous and is not accepted as calibrated
  motion.
- The first trial must not be expanded beyond the existing 20 mm/s, 400 ms
  one-shot envelope.  A later distance increase requires a separate review and
  approval.
