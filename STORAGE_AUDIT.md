# Storage audit

Audit date: 2026-10-20 JST  
Scope: `/Users/imaichika/robot` and read-only Colima metadata  
Safety: no hardware access, no TCP 9000 connection, no move/delete/prune, and no existing source edits.

## Executive result

The repository occupies about **7.94 GiB** (allocated size). The dominant item is `.git` at 6.81 GiB. `git count-objects -vH` reports **24 garbage temporary pack files totaling 5.50 GiB**. They are the largest approval-gated cleanup candidate. The next items are `.venv` (0.99 GiB), `runs` (86.8 MiB), `artifacts` (44.8 MiB), and `simulation` (9.8 MiB).

No file was deleted or moved. Existing uncommitted work was preserved. A verified external backup was subsequently written to `/Volumes/名称未設定/robot-backup-20261020`; see `BACKUP_PLAN.md`.

## Approved cleanup result (2026-10-20 JST)

After the verified SSD backup, the owner authorized maximum safe cleanup. The following were removed:

- Git temporary/unreachable objects using `git gc --prune=now`;
- `runs/detect/train/weights/last (1).pt`;
- `runs/detect/taco/weights/args.pt` and `last.pt` (active `best.pt` retained);
- `artifacts/gazebo_litter_pilot_colab.zip`;
- `artifacts/oscillation_diagnostics/events.jsonl`;
- project Python `__pycache__` directories.

Measured allocated-size reduction is approximately **6.87 GiB**. `.git` fell from 7,144,120 KiB to 30,292 KiB; `runs` from 88,904 KiB to 28,164 KiB; and `artifacts` from 45,824 KiB to 22,068 KiB. Git reports one 28.92 MiB pack, zero loose objects and zero garbage after cleanup. `git fsck --full` completed without corruption.

All deleted project files remain recoverable from `/Volumes/名称未設定/robot-backup-20261020`. Docker/Colima, `.venv`, active models, source, tests, calibration and current evidence were not removed.

## Top-level allocated size

| Path | KiB | Approx. |
|---|---:|---:|
| `.git` | 7,144,120 | 6.81 GiB |
| `.venv` | 1,041,916 | 0.99 GiB |
| `runs` | 88,904 | 86.8 MiB |
| `artifacts` | 45,824 | 44.8 MiB |
| `simulation` | 10,032 | 9.8 MiB |
| `yolo26n.pt` | 5,416 | 5.3 MiB |
| `tests` | 888 | 0.87 MiB |
| root `__pycache__` | 548 | 0.54 MiB |
| `pico` | 164 | 0.16 MiB |

Free space at audit time was about 15 GiB on a 228 GiB data volume (93% capacity reported).

## Largest files (top 30)

The first four are environment binaries; model and project files are shown explicitly. Sizes are logical bytes.

| Bytes | Path |
|---:|---|
| 386,016,976 | `.venv/.../torch/lib/libtorch_cpu.dylib` |
| 159,885,664 | `.venv/.../_polars_runtime.abi3.so` |
| 42,303,696 | `.venv/.../cv2.abi3.so` |
| 31,138,192 | `.venv/.../torch/lib/libtorch_python.dylib` |
| 22,227,080 | `artifacts/gazebo_litter_pilot_colab.zip` |
| 20,740,421 | `runs/detect/train/weights/last (1).pt` |
| 20,739,909 | `runs/detect/taco/weights/best.pt` |
| 20,739,909 | `runs/detect/taco/weights/args.pt` |
| 20,710,725 | `runs/detect/taco/weights/last.pt` |
| 10,298,272 | `.venv/.../cv2/.dylibs/libavcodec...dylib` |
| 6,530,264 | `simulation/create3_sim/.../dock/visual.dae` |
| 5,544,453 | `yolo26n.pt` |
| 5,387,589 | `runs/detect/real_finetune/weights/best.pt` |
| 4,954,576 | `.venv/.../cv2/.dylibs/libx265...dylib` |
| 4,852,320 | `.venv/.../cv2/.dylibs/libcrypto...dylib` |
| 4,013,352 | `.venv/.../numpy/_multiarray_umath...so` |
| 3,979,520 | `.venv/.../torch/bin/protoc-3.21.12.0` |
| 3,979,520 | `.venv/.../torch/bin/protoc` |
| 3,938,096 | `.venv/.../cv2/.dylibs/libaom...dylib` |
| 3,168,384 | `.venv/.../cv2/.dylibs/libSvtAv1Enc...dylib` |
| 3,161,360 | `.venv/.../cv2/.dylibs/libavfilter...dylib` |
| 3,031,984 | `.venv/.../PIL/.dylibs/libavif...dylib` |
| 2,986,176 | `.venv/.../cv2/.dylibs/librav1e...dylib` |
| 2,898,656 | `.venv/.../cv2/.dylibs/libtesseract...dylib` |
| 2,726,576 | `.venv/.../matplotlib/ft2font...so` |
| 2,364,224 | `.venv/.../cv2/.dylibs/libavformat...dylib` |
| 2,237,808 | `.venv/.../PIL/.dylibs/libleptonica...dylib` |
| 2,198,747 | `.venv/.../torch/include/ATen/RedispatchFunctions.h` |
| 2,094,030 | `artifacts/oscillation_diagnostics/events.jsonl` |
| 2,019,694 | `.venv/.../fontTools/misc/bezierTools.c` |

## Git state

- Branch: `checkpoint/yolo-integration-pre-floor-20261020`, aligned with its origin at audit time.
- Tracked paths: 361.
- Untracked paths from `git ls-files --others --exclude-standard`: 1,019 (many are files below untracked artifact directories).
- Ignored paths: 22,288, principally `.venv`, caches, model weights and training output.
- Porcelain summary: 14 modified entries, 1 tracked deletion (`pico/blink.py`), and 44 top-level/status untracked entries.
- No commit, checkout, reset, clean, GC, or push was performed.
- Git objects: 529.51 MiB loose, 807.11 MiB packed, and 5.50 GiB garbage. The 24 `tmp_pack_*` files total 5,913,706,784 bytes.

## Dependency and duplication findings

- The current detector path is `runs/detect/taco/weights/best.pt`; it is referenced by `vision.py`, `calibrate_floor.py`, comparison/evaluation scripts, packaging, diagnostics, and validation. Classification: **ACTIVE; do not move**.
- `runs/detect/taco/weights/last.pt`, `args.pt`, and `runs/detect/train/weights/last (1).pt` are not found as runtime defaults. They remain **ARCHIVE_CANDIDATE pending provenance/restore confirmation**.
- `runs/detect/real_finetune/weights/best.pt` may be a newer experiment but is not the default. Classification is **HOLD**, not deletion candidate, until model-selection evidence is confirmed.
- `yolo26n.pt` is a base model and reproducible in principle, but downloading may change or become unavailable. Treat as backup-first **REGENERABLE/HOLD**.
- `goal_math.py` delegates to canonical `navigation.py`; `.gitignore` calls `go_to_goal.py` a superseded duplicate. Preserve until its ignored local copy/provenance is checked.
- Identical images occur in submission frame sequences and diagnostic visualizations. They are evidence, so deduplication or archive is approval-gated.
- `simulation/create3_sim` and `simulation/irobot_create_msgs` are ROS 2/Gazebo dependencies used by the container and launch flow. They are ACTIVE.

## Active reference map

| Area | Active chain | Classification |
|---|---|---|
| YOLO | `vision.py` -> `runs/detect/taco/weights/best.pt`; configs/tests -> controller and target manager | ACTIVE |
| ROS 2/Gazebo | `simulation/sim.sh` -> `compose.yaml` -> `Dockerfile`/`supervisord.conf` -> nodes/configs | ACTIVE |
| Pico/Roomba | ROS bridge -> `roomba_tcp_transport.py` -> Pico relay protocol -> UART; hardware path is locked by defaults | ACTIVE + CRITICAL_TEST |
| ArUco | `aruco_pose_publisher.py` -> `aruco_pose_core.py` + `aruco_pose.yaml` -> localization topic | ACTIVE |
| Odometry | `wheel_odometry_node.py` -> `localization_core.py` -> `localization_fusion_node.py` | ACTIVE |
| Calibration | `calibrate_floor.py`, `calibrate_sim_camera.py`, controller YAML, ArUco YAML, correspondences and marker image | ACTIVE |

The Dockerfile explicitly copies the active bridge, transport, ArUco, localization, odometry, calibration and test-node files. Moving any of these would break image construction.

## Evidence classification

- Current/important: `artifacts/floor_drive_tests`, `artifacts/e2e_real_camera_pico_dry_run`, `artifacts/multi_target_test`, `artifacts/yolo_navigation_test`, `artifacts/known_target_test`, `artifacts/submission`, and `PHASE4C_SAFETY_REPORT.md`. Keep and back up.
- Historical/large: `artifacts/gazebo_litter_pilot_colab.zip` (21.2 MiB), old submission frames, diagnostic visualizations and `oscillation_diagnostics/events.jsonl`. Archive candidate only after the submission evidence set is selected.
- Results may be modified or untracked. Git alone is not a backup for them.

## Docker and Colima

Docker daemon access was unavailable because Colima is stopped; image/volume-level usage is therefore **not measured**. `colima list` reports:

- `default`: stopped, aarch64, configured 100 GiB disk.
- `create3`: broken, aarch64, configured 24 GiB disk.
- Physical allocated use under `~/.colima`: about **34.94 GiB**. Approximate components are default data disk 5.34 GiB, create3 data disk 26.77 GiB, default VM directory 1.09 GiB, and create3 VM directory 1.74 GiB.

These sparse disks have much larger logical sizes than physical use. The create3 profile is project-critical despite its reported broken state. Do not delete either VM, image, or volume automatically.

## Capacity-reduction candidates (approval required)

| Candidate | Potential saving | Class | Restore method / condition |
|---|---:|---|---|
| `.git/objects/pack/tmp_pack_*` (24) | 5.51 GiB | REGENERABLE candidate | First back up repository and run Git integrity checks; remove only with explicit approval using Git-supported maintenance or exact validated paths. |
| `.venv` | 0.99 GiB | REGENERABLE | Recreate from recorded Python version and `requirements.txt`; verify all 248 tests first. |
| Python `__pycache__` outside venv | 1.86 MiB | REGENERABLE | Python recreates it. |
| Non-active training weights | up to ~59.3 MiB for three large files | ARCHIVE_CANDIDATE | Restore from verified model backup; keep `taco/weights/best.pt`. |
| Other `runs` output excluding active best | at most ~67 MiB | ARCHIVE_CANDIDATE | Restore complete `runs` archive and verify SHA-256. |
| Historical artifacts | up to 44.8 MiB | ARCHIVE_CANDIDATE | Restore verified artifacts archive; retain final evidence selection locally. |
| Colima physical allocation | 34.94 GiB total, **not claimed as safely reclaimable** | HOLD | Export/rebuild images and volumes; preserve arm64 ROS 2/Gazebo/create3 environment. |
| Docker images/volumes | Unknown (daemon stopped) | HOLD | Start only with approval, inventory first, export required images/volumes, never prune automatically. |

Cleanup actually recovered approximately **6.87 GiB** after backup and validation. A further 0.99 GiB remains theoretically available from `.venv`, but it is intentionally retained so the final regression suite remains immediately runnable. No TACO source dataset or experiment video was present in this checkout; `dirt_dataset` contains only a 97-byte `.gitkeep`-style placeholder.

## Findings requiring attention, not cleanup

- Static syntax parsing checked 123 project Python files. Existing modified `pico/main.py` fails at line 41 with `unexpected indent`.
- Existing modified `pico/check.py` contains a non-zero Drive Direct command. It was not run. Its role should be confirmed before final deployment.
- Secret-keyword scan found only documentation/config code references to Wi-Fi/SSID/password; no value was copied to a backup. A real backup must still run a secret scanner and manually inspect Pico deployment copies.
- The documented historical baseline is 249 passing tests, but the current tree contains and runs 248 tests. No test was moved or deleted by this audit.

