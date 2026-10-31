# 最終統合試験チェックリスト

この文書は次回の有人単発試験用であり、実行を自動承認しない。異常時はその試行を中止し、再ARM・再Driveしない。

## 固定構成

- branch `fix-arm-ack-session-recovery`、試験前に`git rev-parse HEAD`を記録
- model `runs/detect/real_finetune/weights/best_position_v2_20261028.pt`
- SHA256 `3e1de3c72c6d807fd527cefc6b70801275e3a9b54bb9daa8d87a59dc7be12b03`
- confidence `0.50`、imgsz `960`、USB camera `0`、800×600
- ArUco DICT_4X4_50 / ID 0 / 80 mm、`simulation/camera_calibration_20261028.json`
- 管理された検出範囲: 前方60–80 cm。カメラを動かしたら較正は無効

## 非走行プレフライト

- [ ] `git status --short`で既知のユーザー変更を識別し、試験コードへ混入させない
- [ ] `shasum -a 256`でmodelを照合
- [ ] Gazebo/Create3が停止、実機用コンテナは1組のみ
- [ ] `/roomba/cmd_vel_unstamped` subscriberは物理bridge 1、publisherはadapter 1のみ
- [ ] `/roomba/arm_generation`のpublisher/subscriberを各1と確認
- [ ] `ps`、`docker compose ps`、`ros2 node list`で孤児bridgeを探す。重複時はDriveせず各起動terminalを`Ctrl+C`、残存containerを該当composeの`down`で終了
- [ ] 検出転送の`--simulator`が空文字でない。空なら検出が送信されないためNO-GO
- [ ] bridge起動時に`latency_trace_path`を試行固有の新規pathへ明示（既存ログを上書きしない）
- [ ] `COMMAND_TIMEOUT_MS=500`、`DRIVE_SESSION_TIMEOUT_MS=400`、`ALLOW_NONZERO_DRIVE=False`、`BRC_WAKE_ENABLED=False`
- [ ] host `max_speed_mm_s=20`、`max_drive_session_duration=0.4`、50 cm guard、pose freshnessが既定値
- [ ] STOP ACK `0x07`、左右0、Safe 2
- [ ] Mac–Colima clock skew <1 s
- [ ] ArUco姿勢fresh、YOLO 3連続、独立停止手段、支持/経路安全

## 副作用のない確認

次は読み取り、hash、mock testだけで、ARM/Driveを送らない。

```bash
git rev-parse HEAD
shasum -a 256 runs/detect/real_finetune/weights/best_position_v2_20261028.pt
env ALLOW_NONZERO_DRIVE=False drive_enabled=false control_enabled=false calibration_ready=false .venv/bin/python -m unittest -v
bash -n simulation/sim.sh
```

`real-camera-ai-dry-run-up`は物理出力を持たないが実カメラを使うため有人時だけ起動する。replayは`./simulation/sim.sh odometry-dryrun-e2e`など既存の明示dry-runを使い、新しい実機経路は追加しない。

## 個別承認後だけ行う単発試験

- [ ] 人間が当該試行を明示承認
- [ ] lock変更は試験直前、20 mm/s以下・400 ms以下・1試行のみ
- [ ] ARM ACK完全受信をARM受理と区別
- [ ] host deadline、STOP begin/return、Pico UART STOP、Drive sequence、期限後Drive数を保存
- [ ] 物理停止・異音・再発進を目視
- [ ] 異常時は独立停止/電源断。自動再ARM・再Driveなし

## 終了

- [ ] bridge/adapter/cameraを終了
- [ ] Pico lockをFalseへ戻し保存・再起動
- [ ] STOP ACK `0x07`、左右0、Safe 2
- [ ] Pico無通電、両輪停止
- [ ] ログ・動画・commit SHA・model SHAを同一試行IDで保存

