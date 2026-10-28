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

## STEP 39 position, distance, and shape survey

The final camera and Roomba placement remained fixed. Points outside the
camera field of view (directly left/right of the Roomba) were excluded rather
than counted as detector failures. The attended working-area survey used only
the forward field of view and kept physical output disabled.

| Mask position | Attended result |
|---|---|
| forward 60 cm, centre | PASS: mask-only detection held for 10 s; ArUco ID 0 remained visible |
| forward 60 cm, left 20 cm | PASS: mask-only detection held for 10 s; ArUco ID 0 remained visible |
| forward 60 cm, right 20 cm | FAIL: boundary behaviour / intermittent detection |
| forward 80 cm, centre | FAIL: intermittent detection near confidence 0.50 |
| forward 80 cm, left 20 cm | FAIL: mild but visible detection dropout |
| forward 80 cm, right 20 cm | PASS for one orientation: mask-only detection held for 10 s |
| forward 100 cm, centre | FAIL: no live detection |

At forward 80 cm/right 20 cm, rotating the mask by 90 degrees without moving
its centre reduced detection substantially. Distance alone therefore does not
explain the failures: image position, mask fold/outline, orientation, scale,
and background overlap all contribute. The moving-target/new-target-switch
case remains a separate test and was not scored as a static-position result.

Two attended screenshots captured the same forward-80-cm condition once with
no mask box and once at `litter 0.50`; these are evidence only and were not
used as training images because the overlay is burned into the screenshots.

### Position-diverse raw dataset

Raw overlay-free frames were collected locally under
`datasets/real_position_20261028/`. The repository's existing `/datasets/`
ignore rule remains unchanged, so the image data are not staged for GitHub.

- train: 7 images and 7 positive labels
- val: 1 image and 1 positive label, unseen position/orientation
- test: 1 image and 1 positive label, farther unseen position/orientation
- captured resolution: 1280x960 (the camera did not accept the requested
  1760x1328 in this capture path)
- integrity: all image/label pairs present, normalized boxes valid, no file
  duplicates, and boxes visually checked to include only the mask

Image SHA256 values:

```text
train 7f7f46cd83ca57c41a67686c2c060c13415777fb5c13933977731d730613d3a4 20261028_210000_887016.jpg
train 3cd0d1a2847251babb2fcd7d1330b3b9d75a94129991095bea6fcce7ef3e16db 20261028_210011_410580.jpg
train 929d7cabcec5072941924bc7570629f3cde38a0b5910122eebfc1105426a2bb8 20261028_210028_116550.jpg
train 50cfd185a441089a588c611f3abf54d1f91f4b2b7547bfe96974cfc65f87a743 20261028_210046_204388.jpg
train 84e822d4dc7cc92893f5e63d8f5e7fde18571975fb364bb4aaff90859dd00ed9 20261028_210112_069080.jpg
train bad8e56aab3027ee9c367c4b3f2582bd05cb7d0f1ec473fee29a78edde7fd891 20261028_211329_909089.jpg
train 8cebbd8dfcf72a782f1d7d0d8bce17d730259f80fcd567e8a3b8bfdafc4a2fb6 20261028_211456_389619.jpg
val   2f9ea83793ddf72984cc98be5340a1b69a4c3d8aff6256b8a544159674dbbab0 20261028_210550_717544.jpg
test  e7bd258e3593ecb9156c010dcd9d6ba02182ad331663dc5ebb26004f44891dab 20261028_210905_986544.jpg
```

### Frozen-model baseline on the independent frames

The operating threshold remained 0.50 and `imgsz=640`. The old and new model
files were not modified.

| Model | val mask | test mask | Roomba false positive |
|---|---:|---:|---:|
| old `best.pt` | 0.504 (borderline PASS) | below operating threshold | 0.697 / 0.693 |
| new `best_hardneg_20261028.pt` | 0.619 (PASS) | 0.338 (FAIL) | no corresponding high-confidence FP |

The new model preserves the hard-negative improvement but still misses the
farther independent test mask. This dataset is suitable for a new training
candidate, but the one-image val/test sets are diagnostic evidence, not a
statistically sufficient final accuracy claim. Any retrained model must remain
separate from both frozen models and be evaluated without moving these val/test
images into training.

Repository regression after recording these results: 353/353 tests PASS using
`.venv/bin/python -m unittest discover -s tests -t .`. An earlier invocation
with the system Python was invalid because that interpreter lacked OpenCV and
the managed sandbox denied loopback sockets; it is not counted as a product
test result. At this initial collection checkpoint, no retraining, model
replacement, physical output, commit push, or moving-target switch test had
been performed; the subsequent follow-up is recorded below.

## STEP 39 follow-up training and live-camera result

Three new candidates were trained without overwriting the frozen old or
hard-negative models. The first seven position-diverse training frames were
kept separate from one validation and one test frame. Three additional
100-cm-band frames were later added to training; the original validation and
test frames remained untouched. The updated local dataset contains 10 train,
1 val, and 1 test image. Its transfer ZIP SHA256 is
`e84c040ce56ec0ec22b8cc2545c6d7d16fb9e39edd9267916d5157b4d025c914`.

Candidate results:

| Candidate | Training scope | mixed test mAP50 | real test mAP50 | held-out position test at live settings | Judgment |
|---|---|---:|---:|---|---|
| v1 | real + position only | 0.127 | 0.995 | 0.643 at 640 | reject: catastrophic mixed regression |
| v2 | full mixed + 5x position balancing | 0.641 | 0.982 | 0.123 at 640; 0.630 at 960 | live candidate at 960 only |
| v3 | full mixed + 5x updated position balancing | 0.659 | 0.856 | 0.383 at 640; 0.392 at 960 | reject: held-out distance failure |

The v2 candidate is stored separately as
`runs/detect/real_finetune/weights/best_position_v2_20261028.pt`, SHA256
`3e1de3c72c6d807fd527cefc6b70801275e3a9b54bb9daa8d87a59dc7be12b03`.
It has not replaced the configured model and is not deployed to a physical
output path.

Mac fixed-image timing for v2 was measured over ten post-warmup predictions:

| imgsz | mean wall time | maximum wall time | held-out detection |
|---:|---:|---:|---|
| 640 | 35.98 ms | 38.40 ms | none at confidence 0.50 |
| 960 | 68.98 ms | 69.57 ms | confidence 0.6351 |

An attended live-camera preview then used v2, confidence 0.50, `imgsz=960`,
USB camera index 0, and `--simulator ''`. ROS, Pico, ARM, and physical output
were absent. Results were:

- forward 80 cm at centre, left 20 cm, and right 20 cm: mask-only detection
  held for 10 seconds at each point; ArUco ID 0 remained visible;
- forward 60 cm/right 20 cm: mask-only detection held for 10 seconds;
- mask absent: no `litter` box for 10 seconds; ArUco ID 0 remained visible;
- movement across the 80-cm band from left 20 cm to right 20 cm over about
  three seconds: the box followed the mask with no observed switch to the
  Roomba, floor, or ruler;
- forward 100 cm/right 10 cm: improved response but occasional loss, so this
  distance is outside the qualified working range.

Detector judgment: **PASS only for the controlled forward 60--80 cm working
range with v2 at `imgsz=960`**. The detector movement observation does not
verify SafeTargetManager identity/latch behaviour. Before any physical
AI-linked trial, v2+960 still requires the isolated ROS non-driving E2E for
detection loss/re-detection, ArUco loss, input expiry, STOP candidate, and no
automatic re-ARM. The configured production model remains unchanged until that
gate passes.

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
