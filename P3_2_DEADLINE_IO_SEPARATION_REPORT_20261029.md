# P3-2 ROS 2 deadline STOP / blocking I/O separation report

Date: 2026-10-29 (JST)
Branch: `fix-arm-ack-session-recovery`

## Scope and safety state

This change is limited to host-side deadline enforcement, TCP response waiting,
and timing traces. It does not change the Pico firmware, 400 ms independent
session limit, 500 ms watchdog, ARM generation rules, STOP latch, speed limit,
or BRC policy. No physical Drive was sent during this work. The deployed Pico
must remain `ALLOW_NONZERO_DRIVE=False` until a separately approved attended
test.

## Confirmed cause

The ROS node uses `rclpy.spin(node)`, hence a single-threaded executor. Its 20 Hz
`tick()` callback performed both of these operations synchronously:

1. live Safe-mode request, TCP receive, and Pico UART diagnostic;
2. Drive send followed by a fixed 13-byte ACK receive.

While either receive was blocked, the 100 Hz deadline timer could not run. This
explains why the timer passed in isolation but the 2026-10-29 real session
started host STOP about 181 ms late. The measured STOP TCP write itself was only
0.483 ms and is not the primary source.

New trace stages split the path into `safe_query_start`, Safe request send and
response wait, Drive send, `tcp_drive_ack_wait_start`, ACK receive, and deadline
STOP send/return. A future real trace can therefore assign the delay to Safe
UART response versus Drive ACK rather than treating it as one executor delay.
The historical trace did not contain these boundaries, so its 181 ms cannot be
retroactively divided between Safe response wait and Drive ACK wait without
inventing evidence. What is confirmed from that trace is: the executor did not
run the deadline callback for 181 ms, while the eventual STOP TCP write took
only 0.483 ms. The new instrumentation closes that evidence gap for the next
non-driving and, if separately approved, attended trace.

## Design choice

Three approaches were considered:

- Multi-threaded ROS executor: small code change, but permits concurrent access
  to one request/response stream and can mix a delayed ACK with another request.
- Dedicated I/O worker and priority queue: clean long-term architecture, but a
  blocking worker still needs cancellable receive, and converting connect, ARM,
  diagnostics, Drive, and STOP together is a substantially larger change.
- Deadline-aware receive in the existing single socket owner: selected. It is
  the minimum change that addresses the measured failure without concurrent
  socket owners.

During a bounded session, fixed-response receive uses at most 10 ms polling
intervals. At the absolute deadline, that same owner sends raw STOP, revokes the
local ARM state, closes the connection, and raises `DriveDeadlineExpired`.
Closing is intentional: a late or partial Safe/Drive response cannot be reused
as the response to a later session. Reconnect starts stopped and requires a new
ARM; no Drive is automatically resent.

This establishes host STOP dispatch under a blocked response wait. It does not
by itself prove Pico UART STOP completion or physical stopping; those remain
separate real-device evidence. If the STOP write itself fails, the socket is
still closed (causing Pico disconnect STOP) and the failure is retained in the
exception.

## Verification

Focused no-network tests:

- 50 ms, 181 ms, and 300 ms delayed Drive ACK: deadline STOP began within the
  10 ms polling design bound (test allowance 20 ms).
- Exactly one Drive and one STOP were written; no Drive retry occurred.
- Connection closed, ARM cleared, and sequence reset.
- A delayed response could not enter a later request.
- A live Safe query blocked across the deadline produced the same STOP/close
  behavior before any Drive.
- Source-level bridge deadline and safety checks passed.

Focused deadline/transport/bridge result: **66/66 PASS**.
Canonical project virtual-environment regression: **369/369 PASS**.

Existing regression coverage also retains fragmented responses, response
timeout, disconnect, invalid ACK/CRC, stale ARM generation, incomplete UART
write STOP latch, reconnect reset, STOP diagnostics, and Pico independent
deadline/watchdog behavior.

## Status and next gate

Host design and dry-run: **PASS**.
Real Pico UART STOP timing and physical stopping with this revision:
**UNVERIFIED**. Therefore this result is not an attended-drive PASS.

Before one attended test:

1. Deploy the reviewed host revision and verify Pico remains at 500 ms watchdog,
   400 ms session limit, BRC disabled, and non-zero lock disabled by default.
2. With Drive locked, verify READY, STOP ACK `0x07`, requested wheels `0/0`, OI
   Safe mode `2`, communication quality, clock skew, and one output bridge.
3. Verify the independent physical stop and clear route.
4. Only after a new per-trial human approval, temporarily enable the existing
   20 mm/s, maximum-400 ms single trial.
5. Record host deadline/STOP trace, Pico UART STOP completion, post-deadline
   Drive count, and physical stop; do not re-ARM or re-Drive after any anomaly.

## Follow-up live-camera non-driving E2E

After the code and mock regressions, the isolated
`real-camera-ai-dry-run-up` path was exercised with USB camera 0, model v2,
confidence 0.50, and inference size 960. The runtime contained only webcam,
ArUco, Navigation controller, and dry-run adapter. Gazebo, Create3, Pico, ARM,
and `/roomba/cmd_vel_unstamped` were absent.

- ArUco pose was fresh at about 9.8 Hz; one measured pose was
  `(0.6724, 0.0196)` in `map`.
- Webcam detections were approximately 10 Hz. The placed mask was accepted
  repeatedly at confidence 0.50--0.65 and produced a target near
  `(1.641, 0.198)`.
- A fresh controller trial produced three observed Navigation candidates at
  `linear.x=0.08 m/s`, followed by a zero command. These were ROS dry-run
  candidates and never entered a physical Roomba topic.
- Mask removal held ArUco-only output for 10 seconds with no false litter box.
- Same-position mask re-detection produced no non-zero restart for 6 seconds.
- ArUco loss with the litter detection present produced no non-zero command.
- ArUco restoration recovered a fresh pose near `(0.6732, 0.0180)` but did not
  restart the stopped same target.
- The camera producer and isolated container/network were stopped and removed.

Live-camera non-driving E2E: **PASS**. This confirms detection, pose,
Navigation candidate, detection-loss STOP, pose-loss STOP, and no automatic
same-target restart without enabling any physical output. It does not verify
the revised host STOP timing against a real Pico/UART/robot.

## Locked real-Pico non-driving preflight

The Pico was then checked with Roomba connected but without any non-zero Drive:

- deployed constants: command watchdog 500 ms, session limit 400 ms,
  `ALLOW_NONZERO_DRIVE=False`, and BRC wake disabled;
- Pico `192.168.0.32:9000`, RSSI -39 to -56 dBm;
- Mac ping 10/10, zero packet loss, 3.454/30.541/94.649 ms min/mean/max RTT;
- STOP-only host diagnostic: ACK `0x07`, requested wheels `0/0`, OI Safe `2`;
- one explicitly approved accepted-ARM/no-Drive probe: generation
  `1793226749`, complete accepted ACK, immediate STOP, stale-generation
  rejection, and `nonzero_drive_sent=false`;
- Pico lock restored to `ALLOW_NONZERO_DRIVE=False`, followed by another STOP
  ACK `0x07`, requested wheels `0/0`, and Safe `2`;
- operator confirmed stopped wheels, no abnormal sound, and no restart before
  and after each diagnostic;
- Pico was stopped and unplugged; Docker containers and network were removed.

One first READY receive timed out on the host. A STOP-only retry recovered. The
Pico log showed `READY_TX_OK sent=6` for every related connection and only
`PC closed the connection` disconnects; there was no Pico ACK-generation or
send failure. This is recorded as a transient host/container TCP receive event,
not ignored, and did not authorize ARM or Drive on the failed session.

Real-Pico non-driving communication gate: **PASS with one recovered transient
READY timeout**. Revised deadline STOP under real non-zero Drive remains
**UNVERIFIED**.
