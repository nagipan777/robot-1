# Phase 4C Localization Architecture

## Scope and safety boundary

This phase implements and tests localization without opening TCP 9000, using a
Pico W, writing UART, or sending a Roomba Drive opcode. Existing motion locks
remain authoritative. `hardware-live` is deliberately unavailable.

## Data flow

```text
fixed camera 1280x960
  -> vision.py (ArUco ID/corners in existing detection JSON)
  -> /webcam/detections
  -> aruco_pose_publisher
       -> /robot/pose_candidate          raw valid geometry for inspection
       -> /localization/aruco_pose       fresh, consecutive accepted fixes

mock packet 43/44 counts
  -> /localization/encoder_counts_mock   JSON {left,right,timestamp}
  -> wheel_odometry_node
  -> /localization/wheel_odom            nav_msgs/Odometry, odom->base_link

/localization/aruco_pose + /localization/wheel_odom
  -> localization_fusion_node
       -> /localization/status
       -> /robot/pose                    disabled by default
```

Gazebo simulation continues to use Gazebo `/odom` and `map_frame_adapter`.
Those publishers must not run together with an enabled fusion `/robot/pose`
publisher. The fusion node checks the graph and rejects output unless it is the
only publisher. Its checked-in configuration keeps pose output disabled.

## ArUco estimator and limitations

The detector uses configurable dictionary and marker ID. The publisher checks
1280x960 input, finite non-degenerate corners, minimum pixel area, timestamps,
map bounds, and three consecutive observations. It maps all four corners using
the measured floor homography, then corrects the elevated marker projection:

```text
marker_point = camera_ground + (camera_height-marker_height)/camera_height
                               * (floor_intersection-camera_ground)
```

It applies a configurable marker-center to robot-center offset and normalizes
yaw to `[-pi, pi]`. Current measured configuration is recorded in
`simulation/aruco_pose.yaml`; approved `/robot/pose` output remains off.

No camera intrinsic matrix or distortion coefficients have been measured.
Consequently this is a calibrated 2D floor projection, not a complete 3D
`solvePnP` pose. `marker_size_m: 0.0` explicitly records that the printed side
length is not yet verified. Lens distortion, marker height measurement error,
and oblique-view yaw error remain limitations.

## Encoder and wheel odometry

OI Query List bytes for packet 43 then 44 are `149,2,43,44`; the response is
four big-endian bytes decoded as two unsigned 16-bit counts. This phase only
decodes supplied bytes and mock JSON. It does not add the query to a live Pico
loop. Count differences use signed shortest-distance 16-bit wraparound.

| Parameter | Preview value | Status |
|---|---:|---|
| `encoder_counts_per_rev` | 508.8 | provisional; verify against OI/model |
| `wheel_diameter_m` | 0.072 | provisional; measure loaded wheel diameter |
| `wheel_separation_m` | 0.235 | provisional; measure effective track width |
| `max_tick_delta` | 2000 | conservative software sanity bound; tune from logs |

The first sample establishes a baseline. Non-increasing timestamps, non-finite
timestamps, values outside uint16, or a delta above the configured bound are
rejected without advancing the estimator.

## Fusion algorithm

Wheel odometry predicts `odom->base_link`. An accepted ArUco fix supplies
`map->base_link`. Initialization computes the rigid transform:

```text
map->odom = (map->base_link from ArUco) * inverse(odom->base_link)
```

Prediction composes this transform with the latest odometry. A subsequent
ArUco fix is compared in `map`; position and yaw innovations above configured
limits are rejected. Accepted observations update `map->odom` with a bounded
complementary gain. Thus values from different frames are never directly
averaged.

Quality states are:

- `UNINITIALIZED`: no accepted ArUco alignment.
- `FUSED`: ArUco and odometry are fresh.
- `ODOM_ONLY`: ArUco is temporarily absent; confidence decays.
- `STALE_ODOM`: wheel prediction is stale; no pose is published.
- `LOST`: dead-reckoning limit exceeded; no pose is published.
- `REJECTED`: frame, timestamp, publisher, or innovation check failed.

The default dead-reckoning limit is 2 seconds. When output is eventually
enabled, loss/staleness stops `/robot/pose`; the existing controller pose-age
gate then rejects targets and stops. The existing 0.5 m robot-proximity guard
continues to compare only fresh map-frame target and pose values.

## Modes

- `simulation`: unchanged Gazebo `/odom` plus `map_frame_adapter`.
- `hardware-preview`: mock counts plus camera/synthetic ArUco; no transport.
- `hardware-live`: design only; launcher refuses it in Phase 4C.

## ArUco placement guide

1. Print an unscaled DICT_4X4_50 ID 0 marker and measure its black-square side.
2. Mount it flat and rigidly on top of the Roomba, away from glossy covers.
3. Mark the marker x-axis and record its yaw offset from Roomba forward.
4. Measure marker top-plane height from the floor.
5. Measure marker center to robot center in robot-forward/robot-left axes.
6. Keep the full marker visible across the operating floor; reject partial or
   very small views rather than extrapolating.
7. Validate at independent center/corner positions and at four headings. Record
   position error, wrapped yaw error, jitter, and loss recovery.

## Hardware Go/No-Go

Hardware-live remains No-Go until encoder constants and effective wheel track
are measured, marker size/offset are recorded, packet 43/44 live reads are
repeatable without Drive opcodes, timestamp/jump bounds are tuned from lifted-
wheel logs, independent pose/yaw tolerances pass, exactly one `/robot/pose`
publisher is present, and all existing STOP/watchdog/physical mitigations pass.

## Verification record (2026-10-19)

- ROS-independent localization, safety, vision, transport, and existing
  regression suite: 240 tests passed in the project virtual environment.
- Synthetic DICT_4X4_50 image: ID 0 detected; ID mismatch filtered.
- Container hardware-preview integration: PASS. Mock counts produced
  `odom -> base_link`, fusion reached `FUSED` with confidence 1.0, transport was
  `mock_only`, and `pose_output_enabled` remained false.
- Python compile, shell syntax, and scoped whitespace checks: PASS.
- Real packet 43/44 acquisition, UART, TCP 9000, Pico W, wheel motion, and
  hardware-live fusion: NOT RUN by safety constraint.
