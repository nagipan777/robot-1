# YOLO hard-negative fine-tuning — 2026-10-28

## Scope

The live fixed-camera model classified the ArUco-equipped Roomba as `litter`.
This work changed only the real-webcam training data, Colab fine-tuning notebook,
and macOS camera backend selection. Camera calibration, ROS safety gates, Pico,
ARM, STOP, and drive output were not changed.

## Evidence before training

- Model: `runs/detect/real_finetune/weights/best.pt`
- SHA256: `a1c863beb4db5048eecc34aaf898d7580518219cc6eb4db8f198ec5a0717a68d`
- ArUco-equipped Roomba false positive: initially 0.70–0.75, later 0.50–0.55.
- Mask true positive: 0.82–0.83 initially and 0.77–0.79 after re-presentation.
- Existing excluded Roomba frames contained a mask and were intentionally not
  suitable as empty-label hard negatives.

## Training data and run

- Added five 800x600 current-room images of the ArUco-equipped Roomba with no
  litter present.
- All five labels are empty YOLO label files.
- Packaged effective split counts:
  - train: 53 images (23 positive, 30 negative; 5 older frames excluded)
  - val: 20 images (10 positive, 10 negative)
  - test: 19 images (8 positive, 11 negative)
- The packaged dataset is stored externally as
  `MyDrive/robot-1-colab/real_webcam.zip`, SHA256
  `5069f0f3c5789d1de9234faf8dcc7df0bcaac9651c0bbe859fc2f6cc0b694695`.
- Colab run used the existing single `litter` class and existing settings:
  `imgsz=960`, `batch=4`, `epochs=50`, `patience=10`, `seed=42`.
- Actual trained weight path:
  `real_runs/taco_gazebo_real_hardneg_20261028-2-2/weights/best.pt`.
- New model SHA256:
  `29c2731996fda94b1f810962c67f4ae8f535fff777d57eb8259ce951bd0e20c2`.

## Fixed-test results

- mixed mAP50: `0.6259408076830123`
- real-webcam mAP50: `0.638364185110664`
- real-webcam test: 19 images, 8 instances, 11 backgrounds, 0 corrupt.

Ultralytics appended run suffixes because prior directories existed. The
notebook now evaluates `model.trainer.best` rather than reconstructing a path
from the requested run name, preventing evaluation of an older run.

## Attended live A/B result

The test used only `webcam_bridge` and `aruco_pose_publisher`. There was no
Create3, controller, Pico, ARM, or drive topic.

| Observation | Old model | Hard-negative model |
|---|---:|---:|
| Roomba, mask absent | false positive 0.50–0.55 | no YOLO detection |
| Mask present | 0.77–0.79 | 0.77–0.79 |
| Mask removed | mask detection absent | no YOLO detection; ArUco remained |
| Mask re-presented | detected | mask only detected; ArUco remained |

The hard-negative model therefore removed the observed Roomba false positive
without removing the attended mask detection. This is a non-driving camera
result, not authorization for physical motion.

## Camera acquisition correction

`vision.py` now requests the AVFoundation backend on macOS, matching the
existing dataset capture tool. A temporary 1760x1328 mismatch was correctly
rejected and was traced to the calibrated USB camera being physically
disconnected; the internal camera had been opened instead. No resizing or
recalibration was used.
