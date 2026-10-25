# YOLO → Roomba 680 実機自律移動 最短クリティカルパス

更新日: 2026-10-22

## 現在のdry-run結果

`real-camera-dry-run-up`をPico IPなしで起動した。設定は
`control_enabled=false`、`calibration_ready=false`、`dry_run_enabled=true`で、
`/roomba/cmd_vel_unstamped`は存在せず、TCP 9000にも接続していない。

静止姿勢`map=(1.5, 0.0, 0.0)`と、テストデータであることを明示した合成検出を使用した結果:

- HTTP `/detections` → `/webcam/detections`: 5件
- 画像中心 `(200.0, 700.0)` → map目標 `(1.1076, 0.7028)`
- confidence `0.95`、decision `accepted`
- `/nav/cmd_vel_unstamped`: 非ゼロ8件、ゼロ1件
- `/roomba/cmd_vel_dry_run`: 非ゼロ8件、ゼロ2件
- 非ゼロ値は`linear.x=0.0`, `angular.z=0.8`（旋回判断）
- 検出途絶後にゼロへ復帰

証跡: `artifacts/e2e_real_camera_pico_dry_run/result-A.json`

これはROS経路のPASSであり、YOLO実画像検出または実機Drive成功の証拠ではない。

## Gate 1: 実カメラYOLO dry-run

混合モデル`datasets/PAI_output/best.pt`、1280×960、confidence 0.30を使う。実際のゴミを
カメラ視野へ置き、bbox、confidence、map座標、目標判断、dry-run Twist、検出喪失ゼロを
同一試行で保存する。物理トピックのPublisherは0でなければならない。

合格証拠:

- 実カメラ画像とYOLO bbox
- `/webcam/detections`の実timestamp/confidence/bbox
- accepted目標とmap座標
- `/roomba/cmd_vel_dry_run`の非ゼロおよび最終ゼロ
- `/roomba/cmd_vel_unstamped` Publisher count 0


## Gate 2: Pico配備版とUART部分書込み

配備`main.py`と`pico/pico_relay.py`を認証情報除外後にhash照合する。BRCは未接続、Roombaは
CLEANで手動起床する。

Drive ACK `status=0x01`が複数回再現しているため、これが実機統合の最優先blocker。
STOP ACK `0x07`はこの問題の解消証拠にならない。次段階へ進むには、実際の
`uart.write(Drive Direct 5 bytes)`戻り値と発生条件を車輪浮上状態で取得し、原因に対応した
最小修正後に完全書込みを確認する。部分フレームの追送やDrive全体の自動再送は禁止する。

合格証拠:

- 配備版hash一致
- 全非ゼロsequenceのDrive ACK `status=0x03`
- `status=0x01`、ACK timeout、再接続、Safe喪失が0件
- 通常STOP、watchdog STOP、終了後ACK `0x07`・左右0・Safe=2

## Gate 3: 車輪浮上回帰

両輪浮上、BRCなし、手動起床、左右20 mm/s以下、1秒以下で行う。Gate 2の証拠を取得し、
物理回転、通常STOP、watchdog、再始動なし、旧generation拒否、両ロック復帰を確認する。
1件でもACK異常があればGate 3はFAIL。

## Gate 4: 有人限定床面走行

`ATTENDED_FLOOR_DRIVE_CHECKLIST.md`の全必須項目をPASSにする。全周物理境界、監視者、
単一Publisher、左右20 mm/s以下、最大1秒、単発直進だけを許可する。終了後に両ロックをfalseへ
戻し、STOP限定診断を行う。

## Gate 5: YOLO連動・有人自律試験

Gate 1〜4の全証拠が揃った後だけ実施する。ArUco実カメラ静止姿勢、timestamp鮮度、map frame、
単一`/robot/pose` Publisher、検出喪失停止、50 cm近傍ガード、到達ラッチを先に合格させる。
最初はゴミ1個、閉鎖区域、有人監視、既存の低速制限で行い、異常時は再ARMしない。

Pico電源断またはUART断では新しいSTOPを送れないため、独立停止がない状態での無人運転は
引き続きNO-GO。提出前に目指す実機自律は、物理封じ込めと操作者を伴う限定試験とする。

## 現在の判定

| 段階 | 判定 |
|---|---|
| ROS合成検出dry-run | PASS |
| 実カメラYOLO dry-run | 実施待ち |
| STOP限定実機診断 | PASS |
| UART非ゼロDrive信頼性 | FAIL (`status=0x01`再現) |
| 車輪浮上回帰 | HOLD |
| 有人限定床面走行 | NO-GO（Gate 2未達） |
| YOLO連動・有人実機自律 | NO-GO（Gate 1〜4未達） |
| 無人実機自律 | NO-GO |
