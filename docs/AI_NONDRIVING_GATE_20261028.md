# AI-linked non-driving gate — 2026-10-28

## Scope and isolation

- Base commit: `b253d55`.
- Model under test: `best_hardneg_20261028.pt`, SHA256
  `29c2731996fda94b1f810962c67f4ae8f535fff777d57eb8259ce951bd0e20c2`.
- The old `best.pt` model remains present and unchanged.
- The dedicated runtime contained only `webcam_bridge`,
  `aruco_pose_publisher`, `vision_turn_controller`, and
  `cmd_vel_output_adapter`.
- `control_enabled=false`, `dry_run_enabled=true`, adapter `mode=dry_run`.
- Gazebo, Create3, Pico, ARM, and `/roomba/cmd_vel_unstamped` were absent.

## Attended live-camera results

| Scenario | Result | Evidence |
|---|---|---|
| No mask, ArUco visible | PASS | ArUco ID 0 only; fresh pose candidate/localization; no dry-run command |
| Mask initially presented | SAFETY PASS / DETECTOR FAIL | a 0.666 candidate at 0.4823 m was rejected by the 0.5 m robot-proximity guard; no command |
| Mask removed | PASS | `detections=[]`, ArUco remained fresh, no dry-run command |
| Mask re-presented farther away | SAFETY PASS / DETECTOR FAIL | detections returned, but confidence intermittently approached 0.50 and the safe target latch prevented automatic restart |
| ArUco hidden | PASS | `aruco_detections=[]`, `LOST/marker_lost`, no command |
| ArUco restored | PASS | `REJECTED/warming_up`, then `CANDIDATE_ONLY/pose_output_disabled` after three fresh observations |
| ArUco hidden again | PASS | `LOST/marker_lost`; no automatic physical output |

Two detector false positives remain under the current attended room layout:

- Roomba can be classified as `litter` at confidence 0.52–0.54 while the mask
  is present.
- Floor line tape is intermittently classified as `litter` around the 0.50
  threshold, including when the mask is absent.

Because of these false positives, the live detector quality gate is **FAIL**
for an attended physical AI-linked run. Safety rejection and stop behavior are
separate PASS results and do not turn the detector result into a GO.

## Yellow floor-tape follow-up

The high-contrast line tape was replaced with matte yellow tape closer to the
floor colour. Camera placement and calibration were not changed.

- Three mask-absent ROS samples contained `detections=[]` with ArUco ID 0
  present. No tape or Roomba false positive was observed.
- At a farther mask position, five samples were 0.5503, 0.5028, absent,
  0.5064, and absent. This did not satisfy a stable three-frame gate.
- At a nearer position, three consecutive samples were 0.6221, 0.6789, and
  0.6551. The target was accepted at map `(1.516, 0.441)` and produced only a
  dry-run turn candidate (`angular.z=0.8`).
- Mask removal produced controller STOP and adapter timeout STOP; the physical
  Roomba topic remained absent.
- During attended re-presentation, moving the mask briefly changed the
  candidate enough to be treated as a new target. Dry-run turn candidates were
  generated for about 0.4 s, then stopped on another detection dropout. This
  did not reach hardware, but it means the AI gate alone does not guarantee
  that a moving/repositioned visual target stays latched as the same target.

The tape replacement is therefore an effective controlled-environment fix for
the observed tape false positive. The remaining live blocker is unstable mask
detection versus image position/scale and transient new-target classification
during repositioning.

## Deterministic nonphysical command/timeout result

A synthetic ROS test exercised the same controller and dry-run adapter after
the live producer was stopped. It did not contact Pico or publish a physical
Roomba topic.

```json
{
  "pass": true,
  "evidence_class": "synthetic_ros_nonphysical",
  "physical_output": false,
  "initial_nav_nonzero_count": 19,
  "initial_dry_nonzero_count": 19,
  "nav_stop_latency_s": 1.0332141260000753,
  "dry_stop_latency_s": 1.0348729590000403,
  "same_target_reconfirm_nav_nonzero_count": 0,
  "same_target_reconfirm_dry_nonzero_count": 0
}
```

This verifies stable three-frame detection producing command candidates,
the configured 1.0 s input timeout producing STOP, and same-target
reconfirmation remaining stopped. It is synthetic nonphysical evidence, not a
live-camera detector PASS and not permission for physical motion.

## Human physical-test status and remaining checks

Current judgment: **AI-linked attended physical test remains NO-GO**.

Minimum remaining checks:

1. Keep the final yellow-tape floor layout fixed and improve mask detection
   across the required working area/scale (additional position-diverse positive
   data and mask-plus-Roomba negatives), without changing the 0.50 threshold.
2. Repeat live mask detection/loss/re-detection and ArUco loss with zero false
   targets, while preserving the three-frame gate, 50 cm guard, freshness, and
   no automatic restart.
3. Immediately before any separately approved physical trial, re-confirm
   physical-output exclusivity, STOP ACK `0x07`, requested wheels `0/0`, Safe
   mode `2`, 20 mm/s limit, independent 0.4 s stop, watchdog, and independent
   human stop means.
