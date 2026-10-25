# Test inventory

Audit date: 2026-10-20 JST

## Result

The safe repository-level command completed successfully:

```bash
env ALLOW_NONZERO_DRIVE=False drive_enabled=false control_enabled=false calibration_ready=false \
  .venv/bin/python -m unittest -v
```

Result: **248 tests run, 248 passed, 0 failed, 0 skipped** in 1.527 seconds. Tests used mocks, temporary directories, and loopback dynamic ports. No Pico UART, robot drive, physical camera, or real TCP 9000 connection was used. Printed `192.0.2.1:9000` messages came from mocked test calls using the documentation-only TEST-NET address.

An initial `unittest discover -s tests -v` run produced 23 environment/collection errors: 3 from the root/tests `test_ip` name collision and 20 from sandbox denial of a loopback ephemeral bind. This was not a product failure. Running the documented root command resolved both and passed all tests.

Historical documentation says 249 tests, but the current source contains exactly 248 `test*` methods (244 below `tests/` plus 4 in root `test_navigation.py`). No test was moved or removed during this audit, so the one-test difference predates it and remains a provenance question.

## Classification by test file

### CRITICAL_TEST — safety and physical-output boundaries

- `tests/test_pico_relay.py` (4): watchdog, reconnect, ARM/watchdog extension, readiness ordering.
- `tests/test_pico_relay_protocol.py` (18): ARM generation, ACK, sequence/CRC, stop latch, Safe/OI diagnostics, UART partial writes.
- `tests/test_pico_dry_run_protocol.py` (15): UART-free dry-run protocol, reconnect, timeout, ACK/session/sequence.
- `tests/test_roomba_bridge_tcp.py` (13): `drive_enabled`, Safe gate, stop ACK, reconnect and watchdog launch locks.
- `tests/test_roomba_tcp_transport.py` (24): transport framing, ACK, reconnect, SafeModeGate behavior and wheel ordering.
- `tests/test_roomba_driver.py` (13): UART driver with mocked serial, watchdog, Safe mode, stop-on-error and encoder handling.
- `tests/test_roomba_wheel_lift_test.py` (14): bounded wheel-lift/floor profiles and dual permissions.
- `tests/test_roomba_diagnostic_codec.py` (6): read-only diagnostics and no Drive opcode.
- `tests/test_cmd_vel_output_adapter.py` (4): timeout/stop and physical-topic isolation.
- `tests/test_safe_target_manager.py` (10): target latch and robot/near-field guard.
- `tests/test_uncalibrated_target_guard.py` (1): calibration gate.
- `tests/test_floor_drive_test_log.py` (3): default locked state and evidence output.
- `tests/test_real_camera_pico_e2e_config.py` (6): final dry-run E2E locks and UART-free port 9001.

These files must remain in place.

### CRITICAL_TEST — navigation, kinematics, localization and transforms

- `test_navigation.py` (4) and `tests/test_navigation_compat.py` (1): canonical goal calculation compatibility.
- `tests/test_differential_drive.py` (14): left/right wheel conversion and limits.
- `tests/test_odometry.py` (8): encoder wrap, motion integration and heading.
- `tests/test_localization_core.py` (10): odometry decode/fusion, stale/outlier behavior.
- `tests/test_localization_config.py` (4): node/config wiring and locked live command.
- `tests/test_aruco_pose.py` (9): ArUco geometry, freshness and calibration lock.
- `tests/test_calibrate_floor.py` (2), `tests/test_sim_camera_calibration.py` (4), `tests/test_camera_world.py` (5): coordinate and camera transforms.
- `tests/test_yolo_navigation_integration.py` (2): navigation boundary, latch and safety restoration.
- `tests/test_known_target_test.py` (5), `tests/test_multi_target_test.py` (3), `tests/test_stage5c_target_management.py` (4): current simulation/E2E navigation configurations.

These files must remain in place.

### ACTIVE verification — vision/data/build configuration

- `tests/test_vision_source.py` (10), `tests/test_yolo_diagnostic.py` (4), `tests/test_yolo_holdout_evaluation.py` (2).
- `tests/test_synthetic_detection_integration.py` (5), `tests/test_sim_litter_comparison.py` (2).
- `tests/test_gazebo_litter_dataset.py` (2), `tests/test_collect_dirt_dataset.py` (3), `tests/test_mixed_colab_export.py` (5), `tests/test_prepare_real_webcam_colab.py` (2).
- `tests/test_pai_output_validation.py` (2), `tests/test_simulation_dds.py` (1), `tests/test_oscillation_diagnostics.py` (1).
- `tests/test_ip.py` (3): mocked connection/explicit drive flag behavior.

Do not archive these before final submission because they validate current scripts and output assumptions.

### Manual hardware scripts — not automatic regression tests

The root files `test_drive_distance.py`, `test_drive_encoder.py`, `test_encoders.py`, `test_odometry_live.py`, `test_roomba.py`, `test_turn_90.py`, and root `test_ip.py` are manual utilities, not unittest cases. Several can access serial hardware or TCP when explicitly invoked. Keep them separate from automatic discovery and do not run unattended. Classification: **HOLD / CRITICAL manual evidence**, pending owner confirmation.

Simulation runners named `*_test.py` under `simulation/` are executable integration runners, not unit-test modules. Their Docker/supervisor references make them ACTIVE.

## Final validation performed

- Python syntax: 123 project `.py` files parsed without executing them; one pre-existing error in `pico/main.py:41` (`unexpected indent`).
- Shell syntax: all `simulation/*.sh` passed `bash -n`.
- Imports: the 248-test run imported the active pure-Python modules exercised by regression tests successfully.
- Calibration reads: floor, simulated camera, ArUco and localization config tests passed.
- Pico settings: protocol/bridge/config tests passed with `ALLOW_NONZERO_DRIVE=False`, `drive_enabled=false`, `control_enabled=false`, and `calibration_ready=false` asserted in current configs/tests.
- Startup references: Dockerfile/supervisor/simulation command references are covered by configuration regression tests and passed. Containers were not started.

## Before/after comparison

No test code was moved or deleted, so the audit comparison is:

| Point | Tests | Passed | Failed/error | Note |
|---|---:|---:|---:|---|
| Historical claim | 249 | 249 | 0 | Exact commit/result file not identified |
| Audit initial constrained discovery | 244 | 221 | 23 | 3 import-path + 20 sandbox loopback errors |
| Audit canonical root run | 248 | 248 | 0 | Current baseline |
| After documentation additions | 248 | 248 | 0 | Re-run recorded in final audit verification |
| After approved cleanup | 248 | 248 | 0 | 1.650 s; active model/config/source retained |

Any future archive action must re-run the canonical command before and after. A lower count is a stop condition unless an approved manifest explains every missing test.

