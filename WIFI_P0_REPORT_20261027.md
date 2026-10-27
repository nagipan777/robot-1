# P0 Wi-Fi / locked ARM transport report (2026-10-27)

## Scope and safety state

This was a Pico-only, non-driving investigation on branch
`fix-arm-ack-session-recovery`.  The Roomba and BRC were disconnected,
`ALLOW_NONZERO_DRIVE = False`, and `BRC_WAKE_ENABLED = False`.  The probe sent
only READY handling, legacy zero STOP, and an ARM request expected to be
rejected with status `0x02`; it never constructed or sent a Drive frame.

## Initial network correction

The Mac was initially associated with a different 5 GHz SSID while the Pico was
on the normal 2.4 GHz SSID.  After moving the Mac to the same SSID and rebooting
the router normally, ARP resolved `192.168.0.29` to Pico MAC
`d8:3a:dd:fe:db:91`.  The DHCP table and Pico address agreed.

The deployed Pico source was also found to be stale because it lacked the
current `WIFI_STATUS` and session diagnostics.  The current repository source
was deployed with credentials supplied locally and both motion locks false.

## Measurements

| Condition | RSSI | Locked cycles | READY | ARM reject ACK | Reconnect READY | Timeout / partial |
|---|---:|---:|---:|---:|---:|---:|
| Original placement, baseline | -74 to -77 dBm | 10 | 8/10 | 8/10 | 10/10 | 2 / 0 |
| Original placement, 250 ms pacing | about -76 dBm | 10 | 9/10 | 8/10 | 10/10 | 2 / 0 |
| Original placement, Pico `TCP_NODELAY` A/B | about -76 dBm | 10 | 7/10 | 7/10 | 8/10 | 1 / 0 |
| Improved placement, same A/B build | -50 dBm | 10 | 10/10 | 10/10 | 10/10 | 0 / 0 |
| Improved placement, repository baseline | -57 dBm | 10 | 10/10 | 10/10 | 10/10 | 0 / 0 |

The original-placement ping after router recovery returned 29/30 packets, with
up to 117.9 ms measured latency.  The improved-placement sample still showed
occasional ICMP loss (28/30), so the RF path is improved rather than proven
lossless.

For the final baseline run, mean/max ARM ACK latency was approximately
28.8/101.5 ms.  There were no partial frames and no timeout.  Host evidence is
saved locally in:

- `artifacts/wifi_p0_20261027/mac_arm_ack_lock_10_baseline_rssi_minus57.json`
- `artifacts/wifi_p0_20261027/mac_arm_ack_lock_10_rssi_minus50.json`
- `artifacts/wifi_p0_20261027/mac_arm_ack_lock_10_tcp_nodelay.json`

## Log correlation and conclusion

In the failed original-placement run, the Pico logged complete six-byte READY
and seven-byte ACK sends while the host sometimes received zero bytes before
its deadline.  When READY reached the host, every observed lock-rejection ACK
also arrived.  `TCP_NODELAY` was successfully enabled in its A/B run but did not
improve the result (7/10 versus the earlier 8/10), so that patch was rejected
and removed from the repository and deployed Pico.

The repository-baseline run passed 10/10 after RSSI improved from roughly
-76 dBm to -57 dBm.  The supported conclusion is therefore that placement/RF
quality was the effective change; the evidence does not support an additional
TCP ACK code change.

## Gate decision

P0 communication diagnosis is **PASS with an operational condition**: retain
the tested Pico placement and require RSSI of at least -65 dBm before P1 or any
later motion trial.  A single non-driving diagnostic timeout may return to the
safe initial state and be investigated, but any timeout during an approved
motion trial remains an immediate abort with no automatic ARM or Drive retry.

P1 remains separate.  Before any Drive, reconnect the Roomba only under human
control, keep BRC disconnected and use manual CLEAN wake, then prove STOP ACK
`0x07`, requested wheels zero, OI Safe mode 2, accepted-ARM ACK delivery, and
STOP invalidation.  Passing P0 does not authorize motion.

## Regression status

The focused Pico protocol and locked-probe tests passed 40/40.  The canonical
project virtual-environment command collected all 338 repository tests; 314
completed successfully and 24 loopback transport cases were blocked only by
the restricted runner's `127.0.0.1` bind permission.  This is the same known
sandbox classification recorded by the earlier 338/338 unrestricted run, not
a product assertion failure.  An unrestricted rerun was attempted but the
approval service rejected execution because its usage quota was exhausted; it
must be repeated when that execution path is available.  No code change is
included in this P0 report commit.
