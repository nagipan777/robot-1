# P1 Roomba-connected non-driving report (2026-10-27)

## Configuration

- Branch: `fix-arm-ack-session-recovery`
- Pico IP: `192.168.0.29`
- BRC: disconnected at both Roomba Pin 5 and Pico GP2
- Wake: manual CLEAN
- UART: verified GP4 TX / GP5 RX path through the known-good level shifter
- Tested RSSI: -56 to -52 dBm
- Roomba wheels: physically stopped throughout

## STOP-only baseline

The existing `roomba-tcp-diagnostic` command reported:

- READY and initial STOP: PASS
- STOP ACK: `0x07`
- host sent / Pico validated / UART written: all true
- requested right / left velocity: `0 / 0`
- OI mode: `2` (Safe)
- ROS `drive_enabled=false`

The operator confirmed no rotation, abnormal sound, or restart after the
diagnostic process closed.

## Accepted-ARM non-driving probe

The operator explicitly approved one accepted-ARM diagnostic.  The dedicated
`simulation/accepted_arm_no_drive_probe.py` path never constructs or calls a
Drive command.  Its sequence is READY/STOP, one fresh ARM, complete ACK, legacy
zero STOP, STOP diagnostic, OI mode query, stale-generation rejection, and
final STOP/close.

Measured result:

```json
{
  "arm_ack_accepted": true,
  "generation": 1793092713,
  "nonzero_drive_sent": false,
  "oi_mode": 2,
  "pass": true,
  "requested_left": 0,
  "requested_right": 0,
  "stale_generation_rejected": true,
  "stop_ack": 7
}
```

The operator confirmed both wheels remained stopped with no abnormal sound or
restart.

## Lock restoration and final state

After the probe, the Pico was restored and rebooted with:

```python
ALLOW_NONZERO_DRIVE = False
BRC_WAKE_ENABLED = False
```

The final STOP-only diagnostic again returned ACK `0x07`, requested wheels
`0 / 0`, OI Safe mode `2`, and ROS `drive_enabled=false`.  The process was
closed and the operator confirmed both wheels remained stopped with no restart.

## Verification and gate

- New probe unit tests plus Pico protocol regression: 40/40 PASS.
- No Drive frame was sent by the live P1 probe.
- P1 non-driving communication and safety gate: **PASS**.
- P1 does not authorize P2 motion. P2 still requires a separate process audit,
  independent stop readiness, applicable physical setup, bounded 20 mm/s and
  0.4 s configuration, and explicit approval for that trial.
