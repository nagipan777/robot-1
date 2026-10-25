# PAI Create3 submission trial

- Trial ID: `pai_create3_20261015T163837JST`
- Result: passed
- Model: `datasets/PAI_output/best.pt`
- Confidence threshold: `0.25`
- Initial pose: approximately `(0.0, 0.0, 0.0)`
- First YOLO target: `(0.490909, 0.506796)` m
- Arrival time: `269.354` s
- Fixed-target stopping distance: `0.150985` m
- Post-arrival observation: `3.2` s
- Post-arrival non-zero commands: `0`
- Hazard detections: `0`
- Final command: `(0.0, 0.0)`
- Safety flags restored: yes
- Recording: 290 fixed-camera snapshots at approximately 1 Hz, encoded as one 290 s MP4

The fixed-target stopping distance is calculated against the first accepted
YOLO target. The raw detector continued to publish bbox centers while Create3
entered the image region, but integration target management retained the first
target for control.
