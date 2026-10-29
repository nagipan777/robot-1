# 提出動画 台本（暫定3分50秒）

公式要件が判明した場合は時間・形式をそちらへ合わせる。実機映像がないため、現時点では編集用台本であり正式動画は未完成。

| 時間 | 画面/素材 | ナレーション | 証拠区分 |
|---:|---|---|---|
| 0:00–0:20 | タイトル、簡略構成図 | 「YOLO、ArUco、ROS 2、Pico W、Roomba 680を、安全ゲート付きで統合しました」 | 説明 |
| 0:20–0:50 | `artifacts/submission/...timelapse_10x.mp4`、Create3 frame | 「まずシミュレーションで認識からNavigationまで段階検証しました」 | simulation |
| 0:50–1:20 | calibration画像、YOLO画像 | 「固定800×600カメラを較正し、ID 0姿勢とmask検出を確認しました。保証範囲は前方60～80 cmです」 | real camera / non-driving |
| 1:20–1:45 | ROS topic/log画面、ARM同期dry-run報告 | 「検出と姿勢からNavigation候補を生成し、途絶・姿勢喪失ではSTOPします。新鮮な候補2件を確認した有人承認済み試行だけ、ARMを1回送る同期手順を非走行で検証しました」 | real camera / non-driving |
| 1:45–2:10 | `artifacts/step41/ai_attended_floor_trial_20261028.json`を見やすく表示 | 「10月28日、有人単発でAI経路から20 mm/s駆動に到達し、289 msでSTOP、再発進なしを確認しました」 | attended hardware; 映像未収録 |
| 2:10–2:40 | ARM/TCP時系列図 | 「最初の試験ではPicoがARMを受理してもACKが0 byteでした。send(None)誤認を直し、ACK完全送信後だけARMを確定しました」 | hardware failure + mock/live probe |
| 2:40–3:10 | P3-2 timeline、6 ACK/7 Drive説明 | 「deadline中のACK待ちを中断してSTOPするよう修正。浮上試験でPicoの最初のDriveからSTOPまで224 ms、STOP後Drive 0件でした」 | attended hardware |
| 3:10–3:30 | odometry SVG/CSV | 「車輪odometryは観測用dry-runまでで、実機encoderやNavigation融合は未確認です」 | mock only |
| 3:30–3:50 | 最終判定表 | 「限定条件で統合を実証しました。連続無人走行、一般環境、実機odometryは今後の課題です」 | conclusion |

## 編集注記

- 10/28 AI床面試験と10/29 deadline浮上試験を別テロップにする。
- host後続エラーログ400.734 msをSTOP開始時刻と書かない。厳密STOP beginは未保存。
- ログ改善版は369/369回帰PASSだが追加実機走行なし、と表示する。
- host 6 ACK/Pico 7 Driveは、sequence 7送信済み・ACK待ちをdeadlineで中断したためと説明する。
- 実機映像の代わりにシミュレーション映像を実機として見せない。
- 10/29追加AI統合は最後のSTOPからARMまで約23.26秒空き、Drive 0件だったFAILとして
  隠さず示す。後続の同期補助は実カメラ非走行PASSであり、実機PASSとは表示しない。
- P3-2ログ改善時点369/369、提出証拠作成時点373/373、同期補助追加後380/380を
  それぞれの時点の回帰結果として混同しない。

## 仮編集に使用する確定素材

- simulation動画:
  `artifacts/submission/pai_create3_20261015T163837JST/pai_create3_20261015T163837JST_timelapse_10x.mp4`
- simulation YOLO静止画:
  `artifacts/submission/pai_create3_20261015T160724JST/yolo_detection.jpg`
- 実カメラ較正開始画像:
  `artifacts/camera_calibration_20261028/experiment_start.jpg`
- ArUco map較正点:
  `artifacts/camera_calibration_20261028/map_refined/point_06_validation_x100_y000.png`
- AI有人床面試行ログ:
  `artifacts/step41/ai_attended_floor_trial_20261028.json`
- deadline解析:
  `artifacts/ai_attended_floor_20261029/deadline_analysis.json`
- odometry軌跡:
  `artifacts/odometry_dryrun/e2e/offline/trajectory.svg`
- ARM同期の説明:
  `docs/submission_2026/ATTENDED_NAV_ARM_SYNC_REPORT_20261029.md`

上記だけでsimulation、較正、ログ証拠の仮編集は可能である。ただし実機映像部分は
プレースホルダーとし、静止画から実機走行映像を生成しない。

