# C2 snapshot candidate manifest

This branch is a reconstruction candidate, not a claim that an exact C2 Git
snapshot existed. C2 was completed after commit `c9ad206` while the working
tree was dirty, and no C2 commit, tag, stash, or reflog entry was created.

## Reconstruction basis

- base commit: `c9ad206` (`Design attended continuous drive trials`)
- C2 result report: `docs/C2_CONTINUOUS_DRIVE_GO_NO_GO_20261030.md`
- C1 result report: `docs/C1_CONTINUOUS_DRIVE_RESULTS_20261030.md`
- C2 trial traces included below
- report statement that the test publisher waits for the bridge's validated
  matching ARM generation before starting its Drive window

## Included post-base changes

- `/roomba/arm_accepted_generation` publication after successful Pico ARM ACK
- test publisher wait for the matching accepted generation
- `continuous_straight` profile, 20 mm/s bound, 1--5 second staged bound
- continuous trial launcher and 0.1 second publish period
- tests for these C2 mechanisms

## Explicitly excluded as later or insufficiently attributable

- Pico execution-state ledger and ACK reconciliation
- ACK fault injection
- TCP_NODELAY investigation
- pre-ARM stale-session probe
- continuous-mode live Safe-query bypass used during later C3 work
- physical-container refactor
- managed START/STATUS/STOP lifecycle work
- C3 reports and C3 transport diagnosis

The Pico files remain exactly at `c9ad206`: that commit already contains the
opt-in continuous timeout implementation, with both drive locks defaulting to
False and the 500 ms command watchdog unchanged.

## C2 evidence

| Trial | generation | trace | SHA256 |
|---|---:|---|---|
| 1 | 1793350829 | `artifacts/roomba_trial_sessions/37181-1793350776-4836/latency.jsonl` | `939484e054fce1e0177876481f8a0dcf1078bf5a5f3cf07828a18c65dcb1c377` |
| 2 | 1793351297 | `artifacts/roomba_trial_sessions/40861-1793351263-13771/latency.jsonl` | `16136aa8ab3b328601ba16cfff1785dc40da4592b8697acee61d5290d6a63e78` |
| 3 | 1793351720 | `artifacts/roomba_trial_sessions/44041-1793351687-12691/latency.jsonl` | `81d4552bcf1651e9ec5a8eeab571ed48c55fbc5d344ce6d48b6d425ff476d3b4` |

## Safety defaults in this candidate

- `ALLOW_NONZERO_DRIVE=False`
- `CONTINUOUS_DRIVE_TEST_ENABLED=False`
- `COMMAND_TIMEOUT_MS=500`
- STOP latch, ARM generation, UART error STOP, and explicit attended
  confirmation remain present

## Validation on 2026-10-31

- C2-focused bridge/publisher suite: 45/45 PASS
- full `unittest discover`: 422 collected; 419 PASS and three `tests/test_ip.py`
  errors caused by the discover-time module name collision with root
  `test_ip.py`
- the same three tests run by package name (`python -m unittest
  tests.test_ip`): 3/3 PASS
- behavioral failures: zero
- `bash -n simulation/sim.sh`: PASS
- all three bundled C2 trace hashes: match the result report

The split 419+3 result is recorded instead of presenting the discover command
itself as a clean 422/422 PASS.

## Trust boundary

This candidate is suitable for code review, diffing, and non-driving replay.
It must not be represented as the byte-for-byte source deployed during C2,
because that source was not committed at the time. Any later real-hardware use
requires deployed-source comparison and the normal explicit trial approval.
