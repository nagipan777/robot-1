# Simulated litter YOLO comparison

Fixed weights: `/Users/imaichika/robot/runs/detect/taco/weights/best.pt`
Confidence threshold: `0.5`
Placement: `[1.5, -0.7]`

| Model | Physical size (m) | Colors / shape | Changed pixels | GT box (px) | Background false positives | Best confidence | Best IoU | Classification |
|---|---|---|---:|---|---:|---:|---:|---|
| can | 0.3 × 0.17 × 0.17 | silver, red; crushed horizontal drink can | 686 | [360, 102, 391, 136] | 1 | 0.7045 | 0.0155 | missed; detections are false positives |
| paper | 0.4 × 0.3 × 0.1 | off-white, beige, brown; three overlapping folded sheets | 904 | [360, 100, 391, 139] | 1 | 0.7295 | 0.0182 | missed; detections are false positives |
| bottle | 0.47 × 0.14 × 0.14 | cyan, blue, white; horizontal PET bottle with neck, cap and label | 706 | [360, 99, 393, 143] | 1 | 0.0000 | 0.0000 | missed; no detection |
