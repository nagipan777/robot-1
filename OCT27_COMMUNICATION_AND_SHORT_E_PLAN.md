# 2026-10-27 Communication diagnosis and Short-E plan

## Principles

- Communication diagnosis and physical Short-E have separate GO/NO-GO gates.
- BRC remains disconnected. Wake the Roomba manually with CLEAN.
- Pico `ALLOW_NONZERO_DRIVE=False` and ROS `drive_enabled=false` are the
  defaults for communication diagnosis.
- A single timeout does not end the non-driving investigation. Return to the
  locked initial state, record it, and continue the remaining measurements.
- A timeout or disconnect during an actual Short-E attempt ends that attempt;
  no re-ARM or re-Drive is allowed in the same attempt.

## Afternoon schedule

| Time | Work | Output |
|---|---|---|
| 13:00–13:20 | Power, wiring, deployed-code and dual-lock check | Baseline checklist |
| 13:20–14:20 | Pico-only locked READY/ARM-rejection/reconnect, 10 trials | CSV/JSON timing log |
| 14:20–14:40 | Analyze success rate, timeouts, latency and close reasons | Communication diagnosis result |
| 14:40–15:20 | Focused READY/ACK/send/Wi-Fi instrumentation if needed | Root-cause evidence or clean rerun |
| 15:20–15:40 | Break and Roomba battery check | Battery decision |
| 15:40–16:20 | Wheel-lift setup, manual CLEAN and STOP-only diagnostic | STOP `0x07`, wheels `0/0`, Safe `2` |
| 16:20–16:50 | Non-driving accepted-ARM round trip and immediate STOP | ARM ACK plus STOP evidence |
| 16:50–17:10 | Separate Short-E GO/NO-GO review | Signed decision |
| 17:10–17:30 | Optional single Short-E attempt or additional diagnostics | Trial log or diagnostic log |
| 17:30–18:00 | Restore both locks, STOP diagnostic, power separation | Shutdown checklist |

If the available afternoon ends earlier, omit Short-E before compressing any
diagnostic or shutdown step.

## Phase A — Pico-only locked communication measurement

Roomba and BRC remain disconnected. Run exactly 10 independent sequences:

1. TCP connect and receive six-byte READY.
2. Send STOP.
3. Send one ARM request while `ALLOW_NONZERO_DRIVE=False`.
4. Receive the expected seven-byte `ARM_REJECTED_LOCK` response.
5. Send STOP and close.
6. Reconnect and receive READY, then send STOP and close.

Never send a Drive frame. Record for every trial:

- trial number and timestamps;
- first READY latency and result;
- ARM ACK byte count, status and latency;
- reconnect READY latency and result;
- host exception;
- Pico `send_all()` byte count or exception;
- Pico Wi-Fi state and disconnect reason.

After a timeout, verify the client is closed, Pico remains locked, no output
bridge/adapter exists, and the next trial starts with a new TCP connection.
Continue until all 10 trials are measured unless there is an electrical fault,
unexpected reboot loop, or evidence of a non-zero frame.

### Phase-A interpretation

- `10/10`: clean communication result.
- `9/10` with one isolated timeout: investigation may continue; inspect the
  failed timing and require the final three trials to pass consecutively before
  considering the accepted-ARM phase.
- Two or more timeouts, malformed/partial ACK, Wi-Fi loss, or a timeout in the
  final three trials: communication diagnosis remains useful, but accepted-ARM
  and Short-E are NO-GO. Focus on ACK generation, `socket.send()`, and Wi-Fi.

These thresholds qualify progression; they do not redefine or extend protocol
timeouts.

## Phase B — STOP-only Roomba diagnostic

Only after wheel lift, independent stop readiness, BRC-disconnected wiring and
manual CLEAN wake:

- STOP ACK must be `0x07`;
- requested right and left wheel speeds must both be zero;
- OI mode must be `2` (Safe);
- wheels must remain physically stopped.

Failure keeps physical testing NO-GO but does not erase the Phase-A data.

## Phase C — accepted ARM round trip without Drive

This is distinct from the locked ten-trial measurement. Use the established
safety setup and enable only the minimum ARM path. Do not start the Navigation
adapter and do not publish a Drive command.

1. Confirm zero Drive publishers and stopped wheels.
2. Issue one fresh ARM generation.
3. Require the complete seven-byte accepted ARM ACK at both Pico and host.
4. Immediately send STOP, require STOP ACK `0x07`, and verify wheels `0/0`.
5. Confirm the ARM cannot authorize a later stale command.

An unknown ARM state, ACK timeout or disconnect causes immediate STOP/close and
makes Short-E NO-GO. Do not re-ARM in that attempt.

## Separate decisions

### Communication diagnosis GO

GO means the planned non-driving measurements were safely completed and
produced usable evidence. An isolated timeout does not automatically make this
diagnosis NO-GO.

### Short-E GO

Short-E requires all of the following immediately before the attempt:

- Phase-A progression criteria met;
- Phase-B STOP/Safe diagnostic passed;
- Phase-C accepted ARM and immediate STOP passed without timeout;
- both wheels lifted and stopped, independent stop ready, BRC disconnected;
- fresh ArUco pose, three consecutive YOLO detections and 50 cm guard passed;
- Mac–Colima clock skew below one second;
- one output publisher only, no Gazebo/Create3 output process;
- 20 mm/s maximum, fixed 0.4 s session limit, both 500 ms watchdogs and STOP
  latch unchanged;
- explicit human approval for the attempt.

If GO, perform at most one Short-E attempt. Any ACK timeout, status `0x01`,
pose loss, disconnect, duration violation or abnormal motion ends the attempt.
No re-ARM or re-Drive is permitted in that attempt.

## Shutdown

Restore and verify:

- no non-zero sender;
- ROS `drive_enabled=false`;
- Pico `ALLOW_NONZERO_DRIVE=False`;
- Pico `BRC_WAKE_ENABLED=False`;
- STOP ACK `0x07`, requested wheels `0/0`, OI mode `2`;
- bridge and adapter stopped;
- logs and any video saved;
- Pico and Roomba safely separated as required.

