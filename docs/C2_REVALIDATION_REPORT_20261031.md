# C2 revalidation report (2026-10-31)

## Scope

This revalidation used reconstructed commit
`c1cb75f489a794dcc38de55c806aa54f3f577be1` as the software baseline.
The Pico was also replaced with that commit's `pico/main.py`; only the deployed
Wi-Fi SSID and password were retained. C3-era execution-state reconciliation,
TCP_NODELAY, stale-session probing, and later container lifecycle changes were
not present.

The test condition for every attended trial was:

- wheels fully lifted and support stable;
- one explicit ARM generation;
- left/right request `20/20 mm/s`;
- command window `1.9 s`;
- host and Pico continuous-session limit `2.0 s`;
- Pico inter-command watchdog `500 ms`;
- no automatic re-ARM and no Drive retransmission;
- BRC disabled.

## Execution environment

The full historical Docker image rebuild exhausted the 2 GiB Colima instance
and terminated its daemon. The actual revalidation therefore used the official
`ros:jazzy-ros-base-noble` image with the following files from `c1cb75f`
bind-mounted as the only C2 runtime code:

| File | SHA-256 |
|---|---|
| `simulation/roomba_bridge_tcp.py` | `d9276b32a1c85658ec6d99489d8975279bf3a5502d0b03b4a172835a0647f8b5` |
| `simulation/roomba_wheel_lift_test.py` | `5ed3699efe7e0ce05ded81cd5692d82ff05ae79fabc564aebae1b14c7e46a107` |
| `simulation/roomba_tcp_transport.py` | `27d44b8ed7bc7b51033d6eb9647f91b4424e1898148087ce0bb2442a031bf876` |
| `simulation/latency_trace.py` | `5863e10ab089299044fd2a2c9e49ac2b50d4e5ebed87b274f0e77761f7e9ab52` |
| `simulation/differential_drive.py` | `e11bfe3c43b19b456f448cd089124ef67daf6d6b9ab41c5a89f3b31747999a6e` |
| `simulation/roomba_wheel_lift_test_core.py` | `3d2f4c0be013708b583105b83e22a5650d7bf803def73fe1f1ec555afe304ac9` |

ROS imports passed before hardware access. Each trial had exactly one bridge,
one ARM subscriber, and one command subscriber. The bridge was terminated and
its process absence was verified after every trial.

## Startup diagnostic

The first STOP diagnostic returned `status=0x03` with sentinel wheel values
`32767/32767`; no non-zero Drive was enabled. The automatic reconnect then
confirmed STOP ACK `0x07`, requested wheels `0/0`, and OI Safe mode `2`.
All three trial-specific startup diagnostics subsequently passed on their first
connection.

## Results

| Trial | ARM generation | Drive ACKs | Sequence | Motion | Continuity | Failsafe | Result |
|---|---:|---:|---|---|---|---|---|
| 1 | 1793431569 | 29 | 1-29 continuous | Both wheels observed forward | PASS | STOP and no restart | PASS |
| 2 | 1793431999 | 23 | 1-23 continuous | Physical stop confirmed; direction not separately recorded | FAIL | watchdog/deadline STOP and no restart | FAIL |
| 3 | 1793432533 | 5 | 1-5 continuous | Physical stop confirmed; direction not separately recorded | FAIL | Safe-loss STOP and no restart | FAIL |

### Trial 1

All 29 Drive ACKs had status `0x03` and requested `20/20 mm/s`. The interval
from the first validated Drive ACK to the normal STOP log was approximately
`1.831 s`, below the `2.0 s` fixed limit. Both wheels were observed rotating in
the forward direction. They then stopped with no noise, support movement, or
restart for at least five seconds.

### Trial 2: host scheduling discontinuity

The Pico received sequence 22 and 23 only `135 ms` apart. The host trace shows
that sequence 23's complete ACK arrived `27.607 ms` after its wait began, but
the host did not finish the ACK-validation path until `424.811 ms` after Drive
send. Thus the approximately `391.5 ms` additional delay occurred after the
ACK bytes had reached the host process; it was not a 500 ms gap in Pico Drive
reception and was not TCP transmission time.

No later Drive was dispatched. The Pico watchdog stopped the robot `503 ms`
after its last Drive. Independently, the host deadline STOP began `1.511 ms`
after the 2.0 s deadline and its TCP send returned in `0.561 ms`. Reconnection
then confirmed STOP ACK `0x07`, wheels `0/0`, and Safe mode `2`. Physical stop
and absence of restart were observed.

### Trial 3: OI Safe loss

Five Drive ACKs were validated with status `0x03`. The next live OI query
returned raw `d78201`, i.e. OI mode `1` (Passive). The bridge revoked ARM,
stopped, disconnected, and rejected the publisher's remaining non-zero inputs
as transport-disconnected. Reconnection confirmed STOP ACK `0x07`, wheels
`0/0`, and Safe mode `2`. Physical stop and absence of restart were observed.

## Final classification

- **C2-MOTION:** PARTIAL PASS. Trial 1 demonstrated the intended continuous
  motion; trials 2 and 3 began motion but did not complete the command window.
- **C2-FAILSAFE:** PASS 3/3. Every trial ended stopped, with no automatic
  re-ARM, retransmitted Drive, or physical restart.
- **C2-CONTINUITY:** FAIL 2/3. One host scheduling stall and one OI Safe loss
  prevented repeatable two-second control.
- **C2 overall:** **FAIL / not reproducible at 3/3**. The historical C2 result
  must not be represented as revalidated by this run.

## Non-driving regression

- C2-focused bridge/publisher suite: **45/45 PASS**
  (`python3 -m unittest tests.test_roomba_bridge_tcp
  tests.test_roomba_wheel_lift_test`)
- `bash -n simulation/sim.sh`: **PASS**

An earlier over-broad local command collected 93 tests and produced 27
localhost-bind permission errors from the restricted runner plus one
nonexistent module-name error. Those were test-invocation/environment errors,
not behavioral failures, and are not counted as PASS. The exact historical
C2-focused suite above was then rerun successfully.

## Final safety state

The deployed Pico was returned to:

```python
COMMAND_TIMEOUT_MS = 500
DRIVE_SESSION_TIMEOUT_MS = 400
CONTINUOUS_DRIVE_TEST_ENABLED = False
CONTINUOUS_DRIVE_SESSION_TIMEOUT_MS = 1000
ALLOW_NONZERO_DRIVE = False
BRC_WAKE_ENABLED = False
```

It restarted on `192.168.0.32:9000` with the wheels stopped. No bridge or test
publisher remained. The operator confirmed no noise, support movement, or
restart for at least five seconds.

## Evidence

- `artifacts/c2_retest_20261031/trial1/latency.jsonl`
  (`65fbb4a9d0e28a275acc9951a7d47310c2845d273340bdbb3c260a332cf302ba`)
- `artifacts/c2_retest_20261031/trial2/latency.jsonl`
  (`712b2230e4144868a79bead182ec69c00bff6310b1a59470bb8c482935b65e8a`)
- `artifacts/c2_retest_20261031/trial3/latency.jsonl`
  (`be2efca828f5be37792a4ad8b48056db302cb124e6faf7f1425c9d0655597379`)

The operator-provided Pico trial-2 log is summarized above but was not copied
verbatim into the repository because no source file path was available.
