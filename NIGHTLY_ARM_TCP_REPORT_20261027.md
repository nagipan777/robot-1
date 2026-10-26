# Nightly ARM/TCP report — 2026-10-27

## A. Conclusion

**Partial improvement.** The `0/7` ARM ACK loss has a confirmed software cause:
the Pico non-blocking `send()` returning `None` was treated as a complete write.
The fix is reproduced and passes host-side tests. ACK/session state is now bound
to a connection ID and commits only after all seven bytes are written.

This is not yet “resolved on real Pico.” The reviewed code has not been deployed
and measured against the actual Pico after the final instrumentation changes.

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
  probe with JSON output (not yet run against real Pico).
- `simulation/arm_tcp_mock_stress.py`: identical Mac/Colima mock stress and
  fault injection without any Drive frame.
- Tests: partial ACK sizes 1–6, disconnect during ACK, STOP during ACK, old
  connection rejection, old generation rejection, 499/500 ms boundary, tick
  wraparound, host `0/7` timeout, and reconnect reset.

## D. Test results

- Syntax checks: PASS.
- Focused unit/host/mock regression: **99/99 PASS**.
- Mac normal mock: **300/300 complete**.
- Colima normal mock: **300/300 complete**.
- Fault matrices: expected 25 timeout + 25 partial + 50 complete in each.
- Additional stress: Mac and Colima each passed 5,000/5,000 normal connections;
  each 400-cycle fault matrix classified 200 complete, 100 timeout, and 100
  partial-close cases without invariant violations.
- Real Pico non-driving TCP: **not run**.
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
- Final code on the physical Pico: **unconfirmed/not deployed unattended**.
- Physical Mini-DIN/BRC/power state: based on the last operator report only;
  not independently observable by Codex.

## F. GitHub

- Branch: `fix-arm-ack-session-recovery`
- Base commit already pushed: `5270832aa9bf740ab155a9e4417e511742f23f0a`
- PR: https://github.com/nagipan777/robot-1/pull/3
- The additional connection-ID instrumentation, stress script/results, and
  this report require a follow-up commit/push.

## G. Remaining issues (maximum three)

1. Final code has not been deployed to or measured against the real Pico.
2. Mac-versus-container port-9000 Wi-Fi results, RSSI, firmware, ping loss,
   reset/memory evidence, and real reconnect rate remain unmeasured.
3. Generation state is RAM-only across Pico reboot. The host epoch-based
   generation makes accidental reuse unlikely, but reboot collision behavior
   still requires a documented live test; no protocol persistence was added.

## H. First attended operation

**NO-GO for Mini-DIN reconnection or Short-E movement.** First keep Mini-DIN
and BRC disconnected, then visually confirm the Pico editor still shows
`ALLOW_NONZERO_DRIVE = False`. After that single confirmation, deploy the
reviewed code and run the 10-cycle locked non-driving probe. The communication
gate clears only after its JSON and Pico event log show 10/10 complete ACKs and
10/10 subsequent READY connections with no unexplained reset.
