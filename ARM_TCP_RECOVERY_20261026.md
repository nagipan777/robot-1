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

The Pico now emits monotonic, connection-correlated events for
`TCP_ACCEPT`, `TCP_RECONNECT`, `READY_TX_BEGIN/OK/ERROR`, `ARM_RX`,
`ARM_PENDING`, `ACK_TX_BEGIN/PARTIAL/OK/ERROR`, `ARM_COMMIT`, `ARM_EXPIRED`,
`STOP_LATCHED`, `EXCEPTION`, and `TCP_CLOSE`. The ARM state stores both its
generation and connection ID. A Drive from a different connection ID is
rejected even if its generation and sequence are otherwise valid.

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
- ARM/TCP and related safety suites after connection-ID instrumentation:
  **105/105 PASS**.
- Covered: `send(None)` retry, partial ACK, finite send timeout, ACK failure
  STOP/latch, generation burn/replay rejection, STOP-during-ACK commit denial,
  no-first-Drive 500 ms expiry, host 0/7 timeout reporting, prior-generation
  revocation, reconnect authorization reset, STOP diagnostics, Safe gate, and
  fixed-duration drive-session logic.
- New boundary coverage includes ACK chunk sizes 1 through 6, disconnect after
  three ACK bytes, and rejection of a Drive tagged with the old connection ID.
- `ARM_PENDING` stores its own monotonic reservation time. Commit rechecks the
  connection, generation, state, and the unchanged 500 ms deadline; a delayed
  ACK completion cannot authorize motion.
- Full discovery collected 280 tests. Eight vision/data modules could not load
  because the Mac environment lacks NumPy/OpenCV; three pre-existing
  `test_ip` discovery-name collisions also failed. These are outside the ARM
  path. The two initially affected ARM/READY assertions were corrected and
  are included in the 87 passing tests.

Later final discovery superseded those interim counts: Mac collected 292 tests
with 281 passing and 11 environment/discovery errors. A network-disabled Colima
container with NumPy/OpenCV collected 313 tests with 312 passing. Its sole
remaining error is the container's OpenCV 4.6 lacking the newer ArUco APIs used
by an existing vision test; no speculative vision workaround was retained.

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

The probe attempts a fresh reconnect even after a READY or ARM ACK failure and
records received byte count, failure reason, and READY/ACK/reconnect mean,
maximum, and p95 latency. Correlate its JSON with the Pico console log using:

```sh
python3 simulation/correlate_arm_logs.py pico_console.log \
  artifacts/short_e_wheel_lift/arm_ack_lock_10.json \
  --output artifacts/short_e_wheel_lift/arm_ack_correlation.json
```

The correlator distinguishes no ARM receipt, pre-ACK termination, ACK send
error, Pico send API success without host receipt, and correlated success.

## Mac-native versus Colima mock TCP measurements

These measurements use a localhost mock Pico and must not be presented as real
Pico or Wi-Fi results. No Drive packet is constructed or sent; every ARM reply
is the locked rejection status `0x02`.

| Normal mock measurement | Mac native | Colima/Docker |
|---|---:|---:|
| Cycles / complete ACKs | 300 / 300 | 300 / 300 |
| READY complete | 300 / 300 | 300 / 300 |
| Timeouts | 0 | 0 |
| Mean connect | 0.343 ms | 0.091 ms |
| Max connect | 7.349 ms | 3.088 ms |
| Mean READY receive | 0.217 ms | 0.115 ms |
| Max READY receive | 22.092 ms | 0.468 ms |
| Mean ARM ACK receive | 0.183 ms | 0.101 ms |
| p95 ARM ACK receive | 0.223 ms | 0.160 ms |
| Max ARM ACK receive | 19.772 ms | 0.235 ms |

The separate 100-cycle fault matrix produced the same classifications in both
environments: 50 complete ACKs, 25 deliberate `0/7` timeouts, 25 deliberate
`3/7` close/partial receives, and 100/100 complete READY messages. Thus the
Mac and container clients both distinguish ACK generation from complete host
receipt and recover on the following new TCP connection in the mock setup.

Raw results:

- `docs/arm_tcp_mock_mac_normal.json`
- `docs/arm_tcp_mock_mac_matrix.json`
- `docs/arm_tcp_mock_docker_normal.json`
- `docs/arm_tcp_mock_docker_matrix.json`

Not measured unattended: Pico Wi-Fi connect/READY/ACK latency, RSSI, firmware
version, Pico memory/reboot history, ping loss, and real Mac-versus-container
paths to port 9000. Those require the operator to confirm Pico power, deployed
code, physical disconnection, and locks first.

Additional unrecorded-payload stress runs (summary only) completed after the
JSON baselines:

| Stress run | Mac native mock | Colima/Docker mock |
|---|---:|---:|
| Normal connections | 5,000/5,000 | 5,000/5,000 |
| Normal timeouts / partials | 0 / 0 | 0 / 0 |
| Fault-matrix connections | 400 | 400 |
| Expected complete / timeout / partial | 200 / 100 / 100 | 200 / 100 / 100 |
| Classification invariant violations | 0 | 0 |

## Wi-Fi, TCP, sleep, and recovery audit

- Pico code already attempts `wlan.config(pm=0xA11140)` to disable Wi-Fi power
  saving. It is best-effort because MicroPython support varies by firmware.
- Mac clients set `TCP_NODELAY`; Pico accepted sockets do not. Pico-side
  `TCP_NODELAY` remains a one-variable live experiment, not an unverified code
  change.
- Pico ACK/READY sends have a 100 ms bounded send loop. Initial client data,
  committed-but-unused ARM, and Drive traffic use independent 500 ms checks.
- Every client `finally` path writes STOP, clears pending/armed connection state,
  closes the accepted socket, and records `TCP_CLOSE`.
- The old `send(None)` bug affected both seven-byte ARM ACK and six-byte READY.
  It therefore explains a common mechanism for the observed ARM `0/7` loss and
  subsequent READY `0/6` failures; real-Pico confirmation is still required.
- Colima uses a Docker bridge at `172.18.0.0/16` behind the macOS
  Virtualization Framework. No Docker network settings were changed.
- At audit time macOS AC idle sleep was disabled, but low-power mode was on;
  battery idle sleep was one minute. Incident-time power/sleep history is not
  available, so sleep is not assigned as the root cause.
- The reviewed Pico code emits a best-effort `WIFI_STATUS` event containing
  MicroPython version, RSSI, and free heap. Unsupported fields are reported as
  `unavailable`; SSID and password are never logged.
