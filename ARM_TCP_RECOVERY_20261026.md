# ARM ACK / TCP session recovery — 2026-10-26

## Scope and safety state

This work repairs the failed ARM response path observed during the 2026-10-26
Short-E wheel-lift attempt. It does not authorize another motion test.

- Pico `ALLOW_NONZERO_DRIVE = False` remains the deployment default.
- ROS `drive_enabled=false` remains required for the live non-driving probe.
- BRC is not connected and `BRC_WAKE_ENABLED = False` remains the default.
- The probe sends only READY/STOP/ARM-control traffic. It never constructs or
  sends a Drive frame.
- The 500 ms Pico watchdog, STOP latch, UART failure STOP, 20 mm/s host limit,
  proximity gate, pose freshness checks, and human approval gate are unchanged.

## Observed failure timeline

1. The host received READY, STOP ACK `0x07`, requested wheel velocities `0/0`,
   and OI Safe mode `2`.
2. Pico logged acceptance of ARM generation `1793007225`.
3. The host received `0/7` ARM-response bytes and timed out after 2.008 s.
4. Pico logged STOP and the TCP close. Subsequent host attempts alternated
   READY timeouts and connections closed before READY.
5. No Drive ACK was received and no physical wheel rotation occurred.
6. Shutdown checks restored STOP `0x07`, wheel requests `0/0`, Safe mode `2`,
   and both software locks to false.

## Root cause and bounded contributing factors

### Confirmed root cause

MicroPython may return `None` from `socket.send()` when a non-blocking socket
cannot currently accept data. The old `send_all()` treated `None` as if every
remaining byte had been sent. Pico could therefore log/act on ARM acceptance
while placing zero ACK bytes on TCP. A focused fake-socket regression test
reproduced this as an empty peer buffer.

`send_all()` now retries `None` and would-block results within the existing
100 ms send deadline, returns the actual byte count, and raises on timeout or
other socket errors. Increasing the host ACK timeout was deliberately avoided.

### Contributing factors retained for live measurement

- Every TCP accept performs the bounded OI initialization sequence before
  READY. Rapid reconnect attempts can overlap that roughly one-second setup
  and appear as consecutive host READY failures.
- The former ARM transition had no explicit pending state. STOP or disconnect
  during ACK output could race with the final ARM state transition.

## State and session changes

The Pico state sequence is now:

`STOPPED/LATCHED_STOP -> ARM_PENDING -> ACK completely written -> ARMED`

- A generation is burned when entering `ARM_PENDING`; a failed ACK cannot be
  retried with that old generation.
- ACK write error performs UART STOP and enters `LATCHED_STOP`.
- STOP during ACK clears the pending generation, so post-ACK commit fails and
  cannot revive motion authorization.
- A successfully committed ARM with no first valid Drive expires through the
  existing 500 ms deadline and performs STOP/latch.
- Disconnect always performs STOP/latch and closes the socket.
- The host clears any previous generation before a new ARM attempt and only
  publishes the new generation after validating all seven ACK bytes.

The first-frame deadline after READY is 500 ms. It releases idle diagnostic
sessions so the single-client Pico server can accept a reconnect; it does not
extend any motion authorization or watchdog.

## Diagnostics and tests

`simulation/arm_ack_lock_probe.py` performs repeated locked, non-driving
cycles and optionally writes JSON. Each cycle is:

`READY -> STOP -> ARM_REJECTED_LOCK ACK (7 bytes) -> STOP/close -> reconnect READY -> STOP/close`

Example for the next attended Pico-only test:

```sh
python3 simulation/arm_ack_lock_probe.py 192.168.0.29 --count 10 \
  --output artifacts/short_e_wheel_lift/arm_ack_lock_10.json
```

The command is valid only after visually confirming the deployed Pico still
has `ALLOW_NONZERO_DRIVE = False`; Mini-DIN/BRC must remain disconnected for a
Pico-only run. It does not send a Drive opcode.

Automated non-driving result on 2026-10-26:

- Python syntax compilation: PASS.
- ARM/TCP and related safety suites: **87/87 PASS**.
- Covered: `send(None)` retry, partial ACK, finite send timeout, ACK failure
  STOP/latch, generation burn/replay rejection, STOP-during-ACK commit denial,
  no-first-Drive 500 ms expiry, host 0/7 timeout reporting, prior-generation
  revocation, reconnect authorization reset, STOP diagnostics, Safe gate, and
  fixed-duration drive-session logic.
- Full discovery collected 280 tests. Eight vision/data modules could not load
  because the Mac environment lacks NumPy/OpenCV; three pre-existing
  `test_ip` discovery-name collisions also failed. These are outside the ARM
  path. The two initially affected ARM/READY assertions were corrected and
  are included in the 87 passing tests.

## Remaining attended validation and GO status

Communication-repair status is **code/mock PASS; live Pico validation pending**.
Short-E motion status remains **NO-GO** until all of the following are recorded:

1. Deploy the reviewed `pico/main.py` with both locks false and BRC disabled.
2. Run 10 locked non-driving cycles and save host JSON plus Pico console log.
3. Confirm 10/10 seven-byte rejection ACKs and 10/10 reconnect READY responses;
   investigate any repeating timeout without relaxing deadlines.
4. With Roomba manually awakened, separately repeat STOP `0x07`, wheel `0/0`,
   and Safe mode `2` diagnostics.
5. Only in a later explicitly approved phase, verify an allowed ARM ACK while
   still preventing Drive, then restore both locks false before review.

No actual-Pico test was run unattended because Pico/Roomba power and physical
connections require the operator. No Short-E retry is authorized by this file.
