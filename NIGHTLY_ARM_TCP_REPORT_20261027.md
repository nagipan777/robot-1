# Nightly ARM/TCP report — 2026-10-27

## A. Conclusion

**Locked non-driving Pico path repaired and reproduced.** The `0/7` ARM ACK loss
has a confirmed software cause:
the Pico non-blocking `send()` returning `None` was treated as a complete write.
The fix is reproduced and passes host-side tests. ACK/session state is now bound
to a connection ID and commits only after all seven bytes are written.

The reviewed code was deployed to the actual Pico on 2026-10-27 with both motion
locks false and the Roomba physically disconnected. The locked ten-cycle probe
then passed 10/10. Accepted-ARM and Roomba STOP/OI diagnostics remain separate
attended gates and were not inferred from this result.

## B. Communication comparison

The table is **localhost mock TCP**, not Pico/Wi-Fi performance.

| Measurement | Mac native | Docker/Colima |
|---|---:|---:|
| Normal trials | 300 | 300 |
| READY success | 100% | 100% |
| Complete ARM ACK | 100% | 100% |
| Mean ARM ACK | 0.183 ms | 0.101 ms |
| Maximum ARM ACK | 19.772 ms | 0.235 ms |
| p95 ARM ACK | 0.223 ms | 0.160 ms |
| Timeout count | 0 | 0 |
| New-connection success | 100% | 100% |

The separate fault matrix in each environment ran 100 trials: 25 deliberate
`0/7` timeouts, 25 deliberate `3/7` close/partial responses, and 50 complete
responses were all classified as intended. Actual Pico and ping measurements
are unperformed.

## C. Changes

- `pico/main.py`, `pico/pico_relay.py`: correct `send(None)` handling; explicit
  `ARM_PENDING`; connection-ID binding; complete-write commit; send/ARM expiry
  STOP latch; first-frame deadline; mandatory event logging.
- `simulation/roomba_tcp_transport.py`: separate ARM send, receive, validation,
  and failure events; clear old authorization before every ARM attempt.
- `simulation/arm_ack_lock_probe.py`: repeated real-Pico locked/non-driving
  probe with JSON output and a recovery attempt after failures (not yet run
  against real Pico).
- `simulation/correlate_arm_logs.py`: generation-based Pico/host correlation
  and A/B/C/D failure classification.
- `simulation/arm_tcp_mock_stress.py`: identical Mac/Colima mock stress and
  fault injection without any Drive frame.
- Tests: partial ACK sizes 1–6, disconnect during ACK, STOP during ACK, old
  connection rejection, old generation rejection, 499/500 ms boundary, tick
  wraparound, host `0/7` timeout, and reconnect reset.

## D. Test results

- Syntax checks: PASS.
- Focused unit/host/mock regression: **105/105 PASS**.
- Mac normal mock: **300/300 complete**.
- Colima normal mock: **300/300 complete**.
- Fault matrices: expected 25 timeout + 25 partial + 50 complete in each.
- Additional stress: Mac and Colima each passed 5,000/5,000 normal connections;
  each 400-cycle fault matrix classified 200 complete, 100 timeout, and 100
  partial-close cases without invariant violations.
- Real Pico locked non-driving TCP after deployment: **10/10 PASS**; READY,
  complete seven-byte lock-rejection ACK, and subsequent READY were each 10/10,
  with zero timeout and zero partial receive.
- Real Roomba UART/STOP ACK: **not run**; Mini-DIN was reported disconnected.
- Full discovery on Mac collected 292 tests: 281 passed; eight import errors
  require NumPy/OpenCV and three errors came from ambiguous top-level discovery.
- Full discovery in a network-disabled Colima test container collected 313
  tests: **312 passed**. The one remaining test requires newer OpenCV ArUco APIs
  (`generateImageMarker`/`ArucoDetector`) than container OpenCV 4.6 provides.
  A guessed old-API fallback was rejected after it caused a native exit 139;
  no unrelated vision compatibility change was retained.

## E. Safety

- Repository/deployment defaults: `ALLOW_NONZERO_DRIVE=False`,
  `BRC_WAKE_ENABLED=False`.
- Non-zero Drive to Pico/Roomba overnight: **none**.
- Mock non-zero values exist only inside isolated unit tests.
- No ROS bridge, output adapter, Gazebo, or test client process was observed;
  Colima and an interactive container shell remained running.
- Final code on the physical Pico: deployed and normalized-hash verified. After
  replacing only the local credential values with empty strings, the deployed
  file and `pico/main.py` both hashed to
  `375482434f47fcf8669e814b8a9dd685753d1039cf733b8d9b7e2a2ec46d0ff3`.
- Pico and ROS non-zero locks remained false. No Drive frame was sent.
- Roomba/Mini-DIN was operator-confirmed disconnected, so STOP ACK and OI mode
  were intentionally not tested.

## F. GitHub

- Branch: `fix-arm-ack-session-recovery`
- Base commit already pushed: `5270832aa9bf740ab155a9e4417e511742f23f0a`
- PR: https://github.com/nagipan777/robot-1/pull/3
- Connection-ID instrumentation and stress evidence: `ffa1f4d`.
- P3–P7 validation follow-up: `bf539f4`.
- Attended Pico diagnostic preparation: `d4acf7f`.
- All three follow-up commits are pushed to the branch above. The working tree
  was clean and matched `origin/fix-arm-ack-session-recovery` when rechecked on
  2026-10-26 (Asia/Tokyo).

## F.1 Final host-side recheck

The focused non-driving suite was rerun with Python `unittest` after the final
push: **131/131 PASS** in 0.301 s. The suite covered the Pico protocol, host TCP
transport, ROS bridge gates, locked ARM probe, log correlator, dry-run protocol,
diagnostic codec, and Roomba driver. The initial sandboxed attempt could not
bind a loopback socket (`PermissionError: [Errno 1]`); rerunning the identical
suite with loopback networking permitted passed completely. No real Pico IP,
ARM request, or Drive command was used in this recheck.

## G. Remaining issues (maximum three)

1. Accepted-ARM ACK is not yet measured on real Pico; doing so requires a
   separately controlled temporary lock enable and does not authorize Drive.
2. Roomba STOP ACK `0x07`, requested wheels zero, and Safe mode 2 were not
   available while Roomba/Mini-DIN was disconnected.
3. Generation state remains RAM-only across Pico reboot. The host epoch-based
   generation makes accidental reuse unlikely, but reboot collision behavior
   still requires a documented live test; no protocol persistence was added.

## H. First attended operation

**Still NO-GO for Short-E movement.** The locked communication gate is now
10/10 PASS, but accepted-ARM ACK and the Roomba STOP/Safe gates remain pending.
Do not infer a movement GO from the Pico-only result.

## I. Real-Pico chronology and measurements

The first live run exposed a diagnostic-tool defect: lock rejection correctly
returned `d7 84 02 00000000`, but the probe incorrectly expected the requested
generation to be echoed. A lock rejection must not reserve that generation.
The probe now validates status `0x02` and records the unchanged highest accepted
generation separately.

Before final deployment, the old Pico file was 21,179 bytes with SHA256
`1a37c7789f16f8dffa9d42280bacf5b9ac5c3af0efddf497831b05e92ec4236b`.
It did not match the reviewed 28,230-byte repository file. With the corrected
probe, the old deployment passed 7/10 immediate reconnect cycles. A 250 ms
settled comparison passed only 5/10. All received ARM replies were complete;
failures were zero-byte READY timeouts.

During that failure window, a 30-packet ping returned 13 replies (56.7% loss),
with 4.063/86.974/461.440 ms min/mean/max RTT. USB diagnostics reported
MicroPython 1.29.0, RSSI -82 dBm, and 173,536 bytes free heap. This demonstrates
that weak Wi-Fi was also present; it does not negate the confirmed old-code
mismatch.

After backing up the old file, preserving credentials only on the Pico,
deploying the reviewed file, and soft-resetting, the same immediate ten-cycle
probe passed 10/10. Mean/max ARM ACK latency was 96.8/274.5 ms; mean/max initial
READY was 1.232/1.457 s; mean/max subsequent READY was 1.274/1.710 s. No timeout,
partial ACK, Drive frame, or non-zero wheel command occurred.
