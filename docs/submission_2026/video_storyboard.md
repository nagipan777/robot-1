# 提出動画 台本（2026-10-29時点の暫定構成）

> **注意：この台本には、実機映像がない時点の古い記述が残っています。** 実際の提出済み動画そのものの内容を示す確定版ではありません。課題に沿った動画の説明順は[成果要約の「4. 動画で見せる内容」](task_focused_summary.md#4-動画で見せる内容安全検証ではなくタスクを主役に)を優先し、現存素材と一致する範囲で使用してください。

公式要件が判明した場合は時間・形式をそちらへ合わせる。実機映像がないため、現時点では編集用台本であり正式動画は未完成。

| 時間 | 画面/素材 | ナレーション | 証拠区分 |
|---:|---|---|---|
| 0:00–0:20 | タイトル、簡略構成図 | 「YOLO、ArUco、ROS 2、Pico W、Roomba 680を、安全ゲート付きで統合しました」 | 説明 |
| 0:20–0:50 | `artifacts/submission/...timelapse_10x.mp4`、Create3 frame | 「まずシミュレーションで認識からNavigationまで段階検証しました」 | simulation |
| 0:50–1:20 | calibration画像、YOLO画像 | 「固定800×600カメラを較正し、ID 0姿勢とmask検出を確認しました。保証範囲は前方60～80 cmです」 | real camera / non-driving |
| 1:20–1:45 | ROS topic/log画面 | 「検出と姿勢からNavigation候補を生成し、途絶・姿勢喪失ではSTOP、同じ目標は自動再開しません」 | real camera / non-driving |
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

