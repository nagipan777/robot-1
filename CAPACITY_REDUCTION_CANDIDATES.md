# Capacity reduction candidates

Prepared 2026-10-20 JST and updated after owner-approved cleanup. Deleted items remain recoverable from the verified SSD backup.

| Priority | Exact target | Potential saving | Keep/restore rule | Approval state |
|---:|---|---:|---|---|
| 1 | Git temporary/unreachable objects | 6.78 GiB actually recovered | Verified SSD backup + complete Git bundle; cleaned with `git gc --prune=now`; post-cleanup `git fsck` passed | Completed |
| 2 | `.venv` | 1,041,916 KiB (0.99 GiB) | Recreate with Python 3.14.6 + `requirements.txt`; prove 248 tests pass in recreated env | Required |
| 3 | `runs/detect/train/weights/last (1).pt`, `runs/detect/taco/weights/args.pt`, `runs/detect/taco/weights/last.pt` | 59.3 MiB | Restore from `models-and-runs/runs/` on SSD | Completed |
| 4 | old Colab ZIP and oscillation event log | about 23.2 MiB | Restore from `evidence/artifacts/` on SSD | Completed |
| 5 | project `__pycache__` directories | about 1.86 MiB | Regenerated automatically | Completed |
| HOLD | `runs/detect/real_finetune/weights/best.pt` | 5,387,589 bytes | Do not remove until final-model choice is documented | Not proposed |
| KEEP | `runs/detect/taco/weights/best.pt` | 20,739,909 bytes | Active default referenced by runtime/tools | Never as cleanup candidate |
| HOLD | `~/.colima` | 36,636,712 KiB physical (34.94 GiB) | Preserve arm64 ROS 2/Gazebo/create3 state; inventory/export before any lifecycle action | Not proposed |
| UNKNOWN | Docker images/volumes | Not measurable while daemon stopped | Inventory only; no prune, volume delete, or VM delete | Not proposed |

Total measured recovery was approximately **6.87 GiB**. `.venv` remains the largest additional regenerable candidate (0.99 GiB), but is retained to preserve immediate testability before final submission.

TACO source data and experiment videos were not found in this checkout. Dataset search found only the 97-byte `dirt_dataset` placeholder. Check external notebooks/download directories before concluding those data do not exist elsewhere.

