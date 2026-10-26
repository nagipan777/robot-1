# BRC non-driving comparison

Date: 2026-10-26

## Scope and invariants

This comparison investigates TCP READY and ARM ACK reliability only. It does
not authorize wheel motion. Both Pico and ROS drive locks remain false, no
non-zero Drive frame is sent, and ARM/watchdog/STOP-latch limits are unchanged.

## Baseline: BRC disconnected

- UART: Pico GP4 TX and GP5 RX through the previously validated level shifter.
- Wake: manual CLEAN button.
- Initial STOP diagnostic passed with ACK `0x07`, requested wheel speeds `0/0`,
  and OI Safe mode `2`.
- The failed Short-E attempt showed Pico `ARM accepted` but host `0/7` ARM ACK
  bytes. The relay's non-blocking `send()` handling was found to treat `None`
  as a successful full write.
- After correcting that defect with the drive lock false, a locked ARM probe
  received all seven response bytes (`status=0x02`) and a reconnect READY.
- A later reboot probe timed out at READY while Pico logged that it had reached
  `relay ready`; READY delivery therefore remains a separate item to trace.

## Proposed BRC wiring

The direct Roomba Pin 5 to Pico GP2 connection was rejected. The attempted
level-shifted topology was:

```text
Pico GP2 -> LV3 | bidirectional level shifter | HV3 -> Roomba Mini-DIN Pin 5
Pico Pin 36 (3V3 OUT) -> LV
Pico Pin 40 (VBUS 5V) -> HV
Pico GND = level-shifter GND = Roomba Mini-DIN Pin 6/7
```

Mini-DIN battery pins 1 and 2 were not used.

The Roomba 600 OI specification identifies Pin 5 as BRC and states that three
50–500 ms low pulses can select 19200 baud. The Pico W GPIO rail is fixed at
3.3 V, so direct connection to a potentially 5 V signal was not accepted.

References:

- https://marsuniversity.github.io/ece387/iRobot_Roomba_600_Open_Interface_Spec.pdf
- https://datasheets.raspberrypi.com/picow/pico-w-datasheet.pdf

## BRC-connected attempt

With Pico USB disconnected, connecting the Mini-DIN caused the Roomba CLEAN
display to illuminate. This was unexpected because the level shifter's LV and
HV supplies were off. It indicates that this converter/channel does not leave
BRC in a safe inactive state while unpowered. The attempt was stopped before
TCP, READY, ARM, diagnostics, or Drive. Pin 5/HV3 and GP2/LV3 were removed.

Result: **FAIL / unsuitable electrical topology**. No comparison claim about
communication stability can be made from this attempt.

## Software safeguard

`BRC_WAKE_ENABLED` now defaults to `False`. If explicitly enabled for a future
electrically qualified interface, a 200 ms low wake pulse is emitted at most
once per Pico boot. TCP reconnects cannot generate the three-pulse baud-change
pattern. Per-connection initialization delay remains unchanged.

## Requirement before another BRC comparison

Use an interface that guarantees BRC remains inactive/high-impedance while
either side is unpowered (for example, a reviewed transistor/open-drain stage
with appropriate pull-up and isolation). Verify idle and low voltages with a
meter before attaching Pin 5. The current bidirectional level-shifter channel
must not be reused for BRC without that verification.

