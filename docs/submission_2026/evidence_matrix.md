# PAI最終課題 証拠対応表

更新日: 2026-10-29 / ブランチ: `fix-arm-ack-session-recovery` / 基準HEAD: `bb48c0a`

## 証拠区分

| 区分 | 意味 |
|---|---|
| H | 人間立会いの実機試験で確認 |
| C | 実カメラ・非走行ROS 2で確認（実機出力なし） |
| M | モック、replay、dry-run、シミュレーションのみ |
| U | 未検証、または必要な直接証拠が未保存 |

## 主張と根拠

| 主張 | 区分 | 判定・条件 | 一次証拠 |
|---|---:|---|---|
| 2026-10-28 AI連動床面駆動 | H | PASS。YOLO→ArUco→Navigation→ARM→Pico→Roomba、20/20 mm/s、最初のDriveから通常STOPまで289 ms、物理停止・再発進なし | `artifacts/step41/ai_attended_floor_trial_20261028.json`、`AI_ATTENDED_FLOOR_DEADLINE_REPORT_20261029.md` |
| 2026-10-29 P3-2 deadline修正版 | H | 両輪浮上でdeadline中のACK待ちを中断しSTOP。Pico上の最初のDriveから通常STOPまで224 ms、STOP後Drive 0件、物理停止 | `P3_2_DEADLINE_IO_SEPARATION_REPORT_20261029.md` |
| P3-2の厳密なhost STOP開始 `<=400.000 ms` | U | 当該試行は`latency_trace_path`未指定。後続エラーログは開始から400.734 msだが、その前にSTOP送信・close済み。STOP開始の小数msは未保存 | 同上 |
| deadline通常ログ改善 | M | 66/66 focused、369/369 full回帰PASS。改善版で追加実機Driveなし | 同上、commit `4ee0270` |
| ARM ACK消失の原因 | H+M | `socket.send()`の`None`を完了と誤認。全byte送信、ARM_PENDING、ACK後commit、接続IDで修正。実Pico locked probe 10/10 | `ARM_TCP_RECOVERY_20261026.md`、`NIGHTLY_ARM_TCP_REPORT_20261027.md` |
| STOP/Safe診断 | H | STOP ACK `0x07`、要求左右0、OI Safe 2を有人で反復確認 | `P3_2_DEADLINE_IO_SEPARATION_REPORT_20261029.md` |
| 実カメラYOLO | C | 固定環境、前方60–80 cm、confidence 0.50、imgsz 960で限定PASS | `P3_2_DEADLINE_IO_SEPARATION_REPORT_20261029.md`、commit `bb48c0a` |
| ArUco ID 0姿勢 | C | 800×600固定カメラ、DICT_4X4_50、80 mm marker。較正独立点誤差0.0427/0.0243 m | `docs/CAMERA_CALIBRATION_20261028.md` |
| 検出途絶・姿勢喪失STOP候補 | C | physical topicなしでSTOP、同一目標の自動再開なしを確認 | `P3_2_DEADLINE_IO_SEPARATION_REPORT_20261029.md` |
| Create3/Gazebo統合 | M | シミュレーション映像・フレームあり。実Roomba証拠とは分離 | `artifacts/submission/`、`PAI_SUBMISSION.md` |
| `/roomba/odom` | M | ROS dry-run E2E PASS、TFなし、Navigation未接続 | `ODOMETRY_DRYRUN_REPORT_20261027.md` |
| 実機encoder 43/44取得 | U | Pico UART経路へ未統合。符号、scale、実機タイミング未確認 | 同上、`WHEEL_MOTION_MONITOR_REPORT_20261029.md` |
| 車輪監視警告 | M | 直進・旋回・惰性・無変化・途絶・wrap・再接続を模擬。警告のみ | `WHEEL_MOTION_MONITOR_REPORT_20261029.md` |

## 2つの有人試験を混同しない

- **10月28日**はAI連動経路の床面単発成功である。Picoログ上のDrive sequence 1–7、20/20 mm/s、Drive→STOP 289 msが保存されている。
- **10月29日**はP3-2 deadline修正版の両輪浮上試験である。hostのブロッキングACK待ち中断を確認し、Pico相対時刻でDrive→STOP 224 msだった。AI連動床面試験そのものではない。

## host 6 ACK / Pico 7 Driveの説明

hostはsequence 1–6の13 byte ACKを完全受信・検証した。その後sequence 7を期限前に送信しACK待ちへ入った。Picoはsequence 7を受信してUARTへ転送したため7件を記録したが、hostはACK待ち中に絶対期限へ達し、STOP送信、ARM失効、socket closeを優先した。このためsequence 7は「Picoで実行されたがhostが完全ACKとして採用しなかった」1件である。期限後に新しいDriveを送った証拠ではなく、遅延ACKを次セッションへ再利用もしていない。

## 最終安全状態

最終確認は`ALLOW_NONZERO_DRIVE=False`、BRC無効、STOP ACK `0x07`、左右0、Safe 2、ROS実機経路停止、Pico無通電、両輪停止・異音なし・再発進なし。これは確認した試行の終了状態であり、無人連続走行の安全保証ではない。

## 2026-10-29提出監査

- 最新ソースの全回帰: **373/373 PASS**。最初のsandbox実行はloopback bindを禁止され24件error、同一コマンドをloopback許可下で再実行して全件PASSした。
- `git diff --check`: PASS。
- 本ディレクトリとhandoff文書のcredential文字列scan: 該当なし。
- 主要artifact SHA256:
  - AI床面JSON: `509868c9f1c1e238769f344f137aa649d52793d550d9a9358094742532d4818d`
  - Create3動画: `7b6fa713a7e34920fa6495e3afa052b332e2802aab3400659ba4496d66229802`
  - deadline解析JSON: `f4dd3f52b9e061346966cca42f6ff3fcf6027611dd00ba0fa58f74d036d8286f`
  - odometry ROS CSV: `6d41aee3f985c7904774ab9bea1c6577895a1626cb8f2318e4ff38b9b5d917ab`
- `PAI_SUBMISSION.md`は2026-10-12時点のCreate3撮影手順であり、最新実機成果を含まない。削除せず履歴資料として残し、本ディレクトリを最終提出準備の基準とする。
- 公式の提出ファイル形式・ページ数・動画時間はリポジトリ内で確認できず、**未確認**。本稿の6–8ページ/3–5分は作業目安で、公式適合を主張しない。

## 2026-10-29追加統合試行

提出準備後の追加有人試行ではARM generation `1793238564`のACKは成立したが、ARM後の
新鮮なNavigation候補がなくDrive 0件、物理移動なしで終了した。再ARMせず、安全設定へ
復帰した。このFAILは10月28日のAI連動床面PASS、10月29日のP3-2浮上PASSを取り消すもの
ではなく、追加試行の操作同期課題として分離する。詳細は
`docs/submission_2026/FINAL_INTEGRATION_ATTEMPT_20261029.md`を参照する。

