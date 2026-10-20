# Stage 5B YOLO diagnostic

Weights SHA256: `7f1e7e5540b30c05ec88cd413516009b6fad155ca654d1b22a324bd45ad39d8d`
Device: `cpu`

## Aggregate metrics

| imgsz | confidence | TP | FP | FN | precision | recall | mean inference ms |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 640 | 0.10 | 7 | 45 | 2 | 0.135 | 0.778 | 172.8 |
| 640 | 0.25 | 4 | 35 | 5 | 0.103 | 0.444 | 134.1 |
| 640 | 0.50 | 0 | 17 | 9 | 0.000 | 0.000 | 172.1 |
| 960 | 0.10 | 9 | 107 | 0 | 0.078 | 1.000 | 348.4 |
| 960 | 0.25 | 8 | 49 | 1 | 0.140 | 0.889 | 353.3 |
| 960 | 0.50 | 5 | 22 | 4 | 0.185 | 0.556 | 358.1 |

## Background versus object-frame false positives

| imgsz | confidence | background FP | object-frame FP |
|---:|---:|---:|---:|
| 640 | 0.10 | 19 | 26 |
| 640 | 0.25 | 18 | 17 |
| 640 | 0.50 | 9 | 8 |
| 960 | 0.10 | 46 | 61 |
| 960 | 0.25 | 20 | 29 |
| 960 | 0.50 | 10 | 12 |

## Object measurements at imgsz=640, confidence=0.10

| kind | variant | GT bbox | width×height px | changed pixels | best confidence | best IoU |
|---|---|---|---:|---:|---:|---:|
| can | far_right | [360, 102, 391, 136] | 31×34 | 686 | 0.579 | 0.015 |
| can | near_left | [225, 324, 251, 353] | 26×29 | 545 | 0.188 | 0.895 |
| can | upper_left_rotated | [191, 161, 220, 193] | 29×32 | 628 | 0.470 | 0.665 |
| paper | far_right | [360, 100, 391, 139] | 31×39 | 904 | 0.642 | 0.018 |
| paper | near_left | [219, 319, 258, 356] | 39×37 | 884 | 0.142 | 0.914 |
| paper | upper_left_rotated | [188, 161, 225, 193] | 37×32 | 890 | 0.159 | 0.884 |
| bottle | far_right | [360, 99, 393, 143] | 33×44 | 706 | 0.128 | 0.784 |
| bottle | near_left | [223, 320, 254, 359] | 31×39 | 601 | 0.425 | 0.575 |
| bottle | upper_left_rotated | [186, 160, 229, 193] | 43×33 | 686 | 0.298 | 0.618 |
