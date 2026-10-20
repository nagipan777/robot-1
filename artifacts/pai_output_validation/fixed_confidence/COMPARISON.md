# Gazebo litter holdout comparison

| model | imgsz | precision | recall | F1 | mAP50 | background FP/image | mean IoU | inference ms |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| best.pt | 640 | 0.180 | 0.444 | 0.256 | 0.240 | 1.444 | 0.770 | 56.1 |
| best.pt | 960 | 0.185 | 0.833 | 0.303 | 0.560 | 2.556 | 0.849 | 77.4 |
| best.pt | 640 | 0.947 | 1.000 | 0.973 | 1.000 | 0.000 | 0.903 | 40.6 |
| best.pt | 960 | 0.878 | 1.000 | 0.935 | 1.000 | 0.000 | 0.906 | 76.9 |
