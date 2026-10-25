# Backup and restore plan

Prepared: 2026-10-20 JST  
Status: **completed and verified** to `/Volumes/名称未設定/robot-backup-20261020`.

## Required destination

Choose an external destination with at least 10 GiB free for project data, or substantially more if Colima exports are included. Prefer an encrypted APFS volume or another access-controlled filesystem. Do not use a public Git remote or shared folder until secrets and personal data have been reviewed.

Set a destination explicitly; do not use an unresolved variable in deletion commands:

```bash
BACKUP_ROOT="/Volumes/PAI_BACKUP/robot-20261020"
mkdir -p "$BACKUP_ROOT"
```

## Backup sets

1. `source-working-tree`: all current source/docs/config, including untracked code and uncommitted changes; exclude `.git`, `.venv`, caches, runs and artifacts.
2. `tests`: `tests/` plus root manual `test_*.py` utilities.
3. `models`: all `.pt/.pth/.onnx/.engine`, especially active `runs/detect/taco/weights/best.pt`.
4. `datasets`: local dataset directories when present. This checkout currently has only the 97-byte `dirt_dataset` placeholder; verify whether TACO/PAI data lives elsewhere.
5. `calibration`: calibration scripts/configs, `aruco_id0.png`, ArUco/localization YAML and correspondence JSON.
6. `evidence`: complete `artifacts/`, floor logs and submission evidence.
7. `environment`: requirements, Dockerfile, Compose, supervisor, Git metadata and exact commit/status records.
8. Optional `git-bundle`: reachable Git history. This does not preserve untracked or ignored files.
9. Optional Colima exports: only after the create3 environment is inventoried and repaired/started with approval.

## Safe copy commands

Run from `/Users/imaichika/robot`. These commands copy; they do not remove source files.

```bash
rsync -a --protect-args \
  --exclude='/.git/' --exclude='/.venv/' --exclude='/**/__pycache__/' \
  --exclude='/runs/' --exclude='/artifacts/' \
  ./ "$BACKUP_ROOT/source-working-tree/"

rsync -a --protect-args tests/ "$BACKUP_ROOT/tests/tests/"
find . -maxdepth 1 -type f -name 'test_*.py' -exec cp -p {} "$BACKUP_ROOT/tests/" \;

rsync -a --protect-args runs/ "$BACKUP_ROOT/models-and-runs/runs/"
cp -p yolo26n.pt "$BACKUP_ROOT/models-and-runs/"
rsync -a --protect-args artifacts/ "$BACKUP_ROOT/evidence/artifacts/"
rsync -a --protect-args dirt_dataset/ "$BACKUP_ROOT/datasets/dirt_dataset/"

git status --porcelain=v1 > "$BACKUP_ROOT/git-status.txt"
git diff --binary > "$BACKUP_ROOT/uncommitted.patch"
git diff --binary --cached > "$BACKUP_ROOT/staged.patch"
git bundle create "$BACKUP_ROOT/robot.bundle" --all
```

Before copying Pico files to any shared destination, inspect deployed/local copies for SSID, Wi-Fi password, IP, tokens and personal data. The repository scan found keyword references but does not prove deployed device files are clean.

## Hash inventory and verification

After copying, create a per-file manifest on the destination:

```bash
cd "$BACKUP_ROOT"
find . -type f ! -name SHA256SUMS -print0 \
  | LC_ALL=C sort -z \
  | xargs -0 shasum -a 256 > SHA256SUMS
shasum -a 256 -c SHA256SUMS
```

Record the output, date, source path and destination volume in `backup-verification.txt`. Success requires every line to report `OK`, the backup to be unmounted/remounted, and a second verification to pass. `archive_manifest.json` contains current source-side group digests for selection/change detection; those are not a substitute for destination verification.

## Restore procedure

1. Restore to a new empty directory, never over the only working copy.
2. Verify `SHA256SUMS` before extraction or use.
3. Clone from `robot.bundle` or copy `source-working-tree`.
4. Apply `staged.patch` and `uncommitted.patch` only after inspecting paths and secrets.
5. Restore models, datasets, calibration and evidence to their original relative paths.
6. Recreate `.venv` with the recorded Python version (audit environment: Python 3.14.6) and `requirements.txt`; do not restore the venv across incompatible machines unless necessary for forensic recovery.
7. Confirm the active model exists at `runs/detect/taco/weights/best.pt` and compare SHA-256.
8. Run static syntax checks and the canonical 248-test command with all safety flags false.
9. Run `bash -n simulation/*.sh`, inspect Docker/Compose references, and keep containers stopped until approved.
10. Never restore Pico deployment files to a device or enable drive flags as part of repository recovery.

## Cleanup gate

No cleanup is authorized until all of these are true:

- destination selected and recorded;
- backup copy completed;
- all destination hashes pass twice;
- source working tree status and binary patches captured;
- active model/calibration/evidence opened from the restored copy;
- 248 tests pass from a restored working copy;
- owner approves an exact path list and expected byte saving.

## Backup integrity result for this audit

Backup completed on 2026-10-20 JST:

- Destination: `/Volumes/名称未設定/robot-backup-20261020`
- Destination filesystem: exFAT
- Files protected by `SHA256SUMS`: 1,596 before the final audit-document refresh
- Logical payload at initial verification: 189,448,010 bytes
- Initial allocated size on exFAT: 385,536 KiB
- Initial `SHA256SUMS` SHA-256: `9ec92987a2057499bd1c6697c7ee5d5de4b1bb7feb35c155f328bcd8d0988e5b`
- Full per-file verification: PASS
- `git bundle verify`: PASS; complete history, 15 refs, HEAD `9bd2c2920d571b0090cdba262a60df59e9f6b6c3`
- Credential-value scan on destination: PASS

Security exception: the local untracked `pico/cmd_vel_dry_run_receiver.py` contained plaintext Wi-Fi credentials. The SSD backup contains a functional copy with `SSID` and `PASSWORD` replaced by empty strings. `SECURITY_EXCLUSIONS.txt` records this. The local source file was not changed.

The final hash-list digest after refreshing these audit documents is stored beside the backup as `SHA256SUMS.sha256` (the checksum list intentionally excludes itself).

