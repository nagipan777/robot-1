# PAI mixed-model domain validation

All rows use the same exported test splits and `imgsz=640`.

| model | domain | precision | recall | F1 | mAP50 | mAP50-95 | inference ms/image |
|---|---|---:|---:|---:|---:|---:|---:|
| taco_original | taco | 0.807 | 0.507 | 0.623 | 0.591 | 0.450 | 40.7 |
| taco_original | gazebo | 0.879 | 0.528 | 0.659 | 0.658 | 0.418 | 37.8 |
| gazebo_only | taco | 0.008 | 0.022 | 0.012 | 0.000 | 0.000 | 35.7 |
| gazebo_only | gazebo | 0.998 | 1.000 | 0.999 | 0.995 | 0.822 | 35.9 |
| taco_gazebo_mixed | taco | 0.777 | 0.420 | 0.546 | 0.476 | 0.326 | 50.1 |
| taco_gazebo_mixed | gazebo | 0.997 | 1.000 | 0.999 | 0.995 | 0.840 | 37.9 |
