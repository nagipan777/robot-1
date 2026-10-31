# P2 attended floor trial report (2026-10-27)

## Authorization and configuration

The operator explicitly approved one attended floor trial at no more than
20 mm/s and a configured 0.4 s session.  The trial used:

- Roomba 680 in a visually cleared, enclosed, level area;
- an operator within immediate reach of CLEAN/power and Pico power;
- BRC disconnected and manual CLEAN wake;
- Pico powered from a mobile battery during the trial, not Mac USB;
- one bridge and one test publisher;
- `max_speed_mm_s=20`, `command_timeout=0.5`, and
  `max_drive_session_duration=0.4`;
- left/right command `20 / 20` mm/s and a single fresh ARM generation.

Before motion, P0 locked communication was 10/10, P1 accepted-ARM/STOP was
PASS, STOP ACK was `0x07`, requested wheels were `0 / 0`, OI mode was Safe
`2`, Mac/Colima measured skew was 0.054107 s, and Gazebo/Create3 and physical
output adapter processes were absent.  The operator confirmed the independent
stop arrangement and physical area.

## Trial evidence

- ARM accepted: generation `1793095961`.
- Bounded session start monotonic: `163453.037197`.
- Six Drive ACKs: every status `0x03`, sequence 1 through 6, right/left
  `20 / 20` mm/s.
- Last ACK monotonic: `163453.479203`.
- Deadline STOP / ARM revoked monotonic: `163453.480528`.
- Normal STOP received/sent monotonic: `163453.481245`.
- Test publisher non-zero message: `linear.x=0.020000`, `angular.z=0.000000`.
- Test publisher reported STOP and exited; no retry or second ARM occurred.
- The approximately 8 mm theoretical displacement was not perceptible to the
  operator.  The operator confirmed complete stop, no abnormal sound, and no
  restart.

## Strict timing interpretation

The configured bound was 0.400 s, but the bridge log interval from bounded
session start to its deadline STOP was:

```text
163453.480528 - 163453.037197 = 0.443331 s
```

Therefore the strict configured-duration criterion is **FAIL by 0.043331 s**.
The transport and physical safety outcome was successful, and the measured
interval from the first completed Drive ACK (`163453.205617`) to deadline STOP
was 0.274911 s, but ACK time is not a substitute for the earlier session-start
timestamp.  This report does not relabel the 0.443331 s interval as a 0.4 s
PASS.  No repeat trial was performed.

The likely implementation contributor is the bridge's 20 Hz supervisory timer
and callback/transport scheduling.  Before another strict 0.4 s trial, the
deadline dispatch must be changed and tested so logged STOP occurs at or before
the required bound under load without weakening the independent 500 ms Pico
watchdog.

## Shutdown evidence

After the trial:

- the bridge was stopped;
- Pico was restored and saved with `ALLOW_NONZERO_DRIVE=False` and
  `BRC_WAKE_ENABLED=False`;
- ROS diagnostic mode reported `drive_enabled=false`;
- the final diagnostic had four preliminary READY/connect timeouts, then
  succeeded with STOP ACK `0x07`, requested wheels `0 / 0`, and OI Safe `2`;
- the operator confirmed complete stop, no abnormal sound, and no restart;
- the Pico program was stopped, UART STOP executed, Mac USB removed, and the
  Pico LED extinguished.

Final hardware state is safe.  The trial is **PASS for ACK/STOP/physical
behavior but FAIL for the strict 0.4 s logged deadline**.  No further physical
motion is authorized from this result.
