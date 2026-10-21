# Phase 4C safety design report

Date: 2026-10-19. Phase 4B passed at 20 mm/s with the drive wheels lifted. This
report does not authorize floor driving. The repository defaults remain
`ALLOW_NONZERO_DRIVE=False`, `drive_enabled=false`, `control_enabled=false`, and
`calibration_ready=false`.

## Stop-path evaluation

| Event | Current response | Evidence | Residual risk |
|---|---|---|---|
| Normal zero Twist | Host emits OI Drive Direct right=0, left=0; ARM/sequence resets | unit and Phase 4A tests | requested speed 0 is not a direct wheel-speed measurement |
| TCP disconnect | Pico `finally` attempts STOP and latches motion; host also attempts STOP on close | unit tests and `disconnect STOP` hardware log | cannot transmit if Pico/UART loses power simultaneously |
| Command silence | Pico writes STOP after 500 ms; host has an independent bounded fallback | mock tests and Phase 4B 503 ms hardware result | up to watchdog latency plus mechanical coast |
| UART partial write | Pico returns negative per-Drive ACK, attempts STOP, and enters `LATCHED_STOP`; host discards authorization | mock test | a broken TX wire can prevent both Drive completion and STOP delivery |
| Pico soft reboot | boot/wait paths attempt STOP, then require READY, STOP diagnostic, Safe and a new ARM | code and mock tests | STOP cannot be emitted during the reset interval |
| Pico power loss | no further UART byte can be emitted | design analysis only | Roomba may retain its last Drive Direct command; physical stop is not guaranteed |
| Roomba safety events | OI Safe may revert to Passive on wheel drop, cliff or charger conditions; host live-mode check revokes authorization | OI Safe gate and tests | Safe mode is not a substitute for an independent emergency stop |

The software therefore cannot claim fail-safe stopping for Pico power loss or an
open Pico-TX/Roomba-RX path. Phase 5 remains No-Go until a wheel-lift-only,
manually approved power-loss experiment or an independent hardware stop/power
cutoff establishes acceptable behavior. No such experiment is automated.

## ArUco pose design

`vision.py` detects the selected marker in the same 1280x960 frame used by YOLO
and appends ordered pixel corners to the existing HTTP payload. The ROS adapter
`aruco_pose_publisher.py` maps those corners with the validated image-to-map
homography, derives yaw from corner 0 to corner 1, and applies two corrections:

1. Marker height: the homography gives the floor intersection of the camera ray.
   Given camera ground point `C`, camera height `H`, marker height `h`, the marker
   point is `C + (H-h)/H * (floor_intersection-C)`.
2. Mount offset: the measured forward/left displacement from marker center to
   Roomba center is rotated by the estimated yaw.

The adapter rejects missing IDs, wrong image resolution, uncalibrated geometry,
non-finite/degenerate corners, small markers, stale/future/non-monotonic stamps,
map-bound violations, and competing `/robot/pose` publishers. It requires three
fresh consecutive detections by default. Marker loss stops publication; the
existing controller then rejects the stale pose and emits STOP.

Defaults publish neither candidate nor approved poses until
`pose_calibration_ready=true`; approved `/robot/pose` additionally requires
`pose_output_enabled=true`. Manual/Gazebo pose publishers must be stopped before
approved output is enabled.

## Measurements required before pose output

- camera optical-center projection on floor: `camera_ground_x/y` [m]
- camera optical-center height: `camera_height_m` [m]
- marker top-plane height: `marker_height_m` [m]
- marker-center to Roomba-center forward/left offsets [m]
- marker x-axis to Roomba forward yaw offset [rad]
- independent position/yaw validation points across the usable floor region

## Phase 5 Go/No-Go

Every item must pass; otherwise the decision is No-Go:

- Pico power-loss and UART-open-circuit stopping risk has an accepted hardware mitigation.
- Physical emergency stop/power isolation and a dedicated operator are available.
- Wheelbase is measured and recorded; direction/sign tests pass.
- Updated sequence/CRC/UART-ACK protocol is deployed on both host and Pico.
- Battery/OI/STOP diagnostics pass immediately and repeatedly.
- ArUco height and center offsets are measured, not assumed.
- Independent ArUco position and yaw validation meets documented tolerances.
- Marker-loss, stale-pose and competing-publisher tests stop control.
- Exactly one approved `/robot/pose` publisher exists.
- YOLO target, robot pose and commands use `map`, metres, radians and fresh timestamps.
- `control_enabled`, `calibration_ready`, `drive_enabled`, and Pico drive permission are enabled only by a new, explicit Phase 5 procedure.
- Test area is physically isolated; floor speed, duration and travel bounds are separately approved.

Until then, Phase 5 floor driving is prohibited.
