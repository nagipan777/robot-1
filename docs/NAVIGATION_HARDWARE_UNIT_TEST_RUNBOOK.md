# Roomba 680 Navigation実機単体試験手順書

更新日: 2026-10-30
対象ブランチ: `feature/navigation-hardware-unit-test`

## 1. 適用範囲と禁止事項

この手順はYOLOを使わず、Navigation計算、Twistから左右輪への変換、Roomba 680の
直進・左右旋回・停止を段階的に確認する。Phase 0のみ実施済みであり、Phase 1以降は
有人実機試験である。

- ARM、非ゼロDriveは各試行の人間による個別承認後だけ許可する。
- 20 mm/s、400 ms、500 ms watchdog、Pico STOP latchを変更しない。
- STOP後に自動再ARM・自動再Driveしない。
- BRCは未接続、CLEAN手動起床を使う。
- Gazebo/Create3と実機出力を同時に有効化しない。
- 実機ブリッジは`roomba-trial-session`の1個だけとし、STOP専用ブリッジを重複起動しない。
- Phase 2の単発方向確認とPhase 4以降の連続閉ループを混同しない。

## 2. 調査結果と再利用範囲

### Navigation計算

- `navigation.calculate_goal()`は距離と目標方位誤差を計算し、誤差を`[-pi,+pi]`へ正規化する。
- `VisionTurnController.update_control()`は`turn_only`と`approach`を実装する。
- `turn_only`は角度誤差だけから`angular.z`を生成する。
- `approach`は角度誤差が`forward_angle_tolerance=0.15 rad`以内のときだけ前進する。
- `arrival_distance=0.15 m`以内、姿勢/目標の期限切れ、検出喪失ではSTOPを出す。
- `SafeTargetManager`はSTOP後の同一目標を`blocked_same`として扱い、自動再開しない。

今回固定する計算パラメータは、既存実カメラdry-run設定と一致する。

| パラメータ | 値 |
|---|---:|
| angular gain | 1.2 |
| max angular speed | 0.8 rad/s |
| angle tolerance | 0.08 rad |
| linear gain | 0.4 |
| max linear speed | 0.08 m/s |
| forward angle tolerance | 0.15 rad |
| arrival distance | 0.15 m |
| wheel separation | 0.235 m |
| hardware wheel limit | 20 mm/s |

### Twistから左右輪

`simulation/differential_drive.py`を再利用する。

```text
left  = (linear.x - angular.z * wheel_separation/2) * 1000
right = (linear.x + angular.z * wheel_separation/2) * 1000
```

ピークが20 mm/sを超える場合は左右を同じ比率で縮小する。したがって、Navigationが
`angular.z=+0.8`を生成した場合は`left=-20, right=+20`、`-0.8`なら
`left=+20, right=-20`となる。

### 実機通信と期限

- `roomba-trial-session`は接続時STOP、STOP ACK、要求速度0/0、Safe 2、ARM待機、
  400 ms host期限、ログ保存、終了STOPを単一ブリッジで行う。
- Picoは別系統の400 ms drive-session期限と500 ms watchdogを維持する。
- 過去の有人浮上試験ではPico上の最初のDriveから通常STOPまで224 ms、STOP後Drive 0件。
- 厳密なhost STOP開始`<=400.000 ms`は当時未保存の試行があるため、今回のtraceで再記録する。

### ArUco

- marker: DICT_4X4_50 ID 0、黒枠80 mm。
- カメラ条件: 800x600、camera height 0.53 m、marker height 0.12 m。
- 出力単位: map frame、m/rad。
- `pose_output_enabled=false`が既定であり、実測時だけ設定と映像を再確認する。
- 較正・分解能以下の変位を成功証拠にしない。

## 3. Phase 0：非走行計算検証（実施済み）

実行コマンド:

```bash
cd /Users/imaichika/robot
.venv/bin/python simulation/run_navigation_hardware_phase0.py \
  --output-json artifacts/navigation_hardware_unit/phase0/result.json \
  --output-csv artifacts/navigation_hardware_unit/phase0/result.csv
```

| ケース | linear.x | angular.z | 左 mm/s | 右 mm/s | 判定 |
|---|---:|---:|---:|---:|---|
| 前方1 m、approach | 0.08 | 0.0 | +20 | +20 | 前進候補 |
| 左90°、turn_only | 0.0 | +0.8 | -20 | +20 | 左回転候補 |
| 右90°、turn_only | 0.0 | -0.8 | +20 | -20 | 右回転候補 |
| 左45°、turn_only | 0.0 | +0.8 | -20 | +20 | 上限飽和 |
| 右45°、turn_only | 0.0 | -0.8 | +20 | -20 | 上限飽和 |
| 距離0.10 m | 0.0 | 0.0 | 0 | 0 | 到達STOP |
| 姿勢途絶 | 0.0 | 0.0 | 0 | 0 | STOP |
| 目標途絶 | 0.0 | 0.0 | 0 | 0 | STOP |
| STOP後の同一旧目標 | 0.0 | 0.0 | 0 | 0 | latch維持 |

PASS条件: JSONの`passed=true`、`arm_sent=false`、`drive_sent=false`。
FAIL条件: 表と異なる符号/速度、20 mm/s超過、喪失時の非ゼロ。
後片付け: なし。実機出力は存在しない。

## 4. Phase 1：実機通信・静止診断

### 前提条件

1. Roombaを安定した支持台へ置き、両輪を完全浮上させる。
2. BRC未接続、UART配線正常、独立停止手段準備済み。
3. Pico配備版で`ALLOW_NONZERO_DRIVE=False`、`BRC_WAKE_ENABLED=False`、
   `DRIVE_SESSION_TIMEOUT_MS=400`、`COMMAND_TIMEOUT_MS=500`を確認する。
4. CLEANで手動起床し、両輪停止を目視確認する。
5. Gazebo/Create3と既存物理ブリッジを停止する。

### 単一ブリッジ起動

Terminal Aで、IPを当日実測値へ置き換えて実行する。

```bash
cd /Users/imaichika/robot/simulation
./sim.sh roomba-trial-session PICO_IP ATTENDED_FLOOR 9000
```

このTerminal AはPhase 2終了まで閉じない。`tee`は使わない。ログはtrial sessionが自動保存する。

必要表示:

- `Connected ... relay READY; STOP sent`
- `STOP confirmed ... ack=0x07`, right=0, left=0
- `Roomba OI mode: 2 (Safe)`

Terminal Bでコンテナへ入り、次を1コマンドずつ確認する。

```bash
cd /Users/imaichika/robot/simulation
./sim.sh shell
```

```bash
pgrep -af 'roomba-bridge-tcp|roomba-encoder-ros-node'
```

```bash
ros2 topic info -v /roomba/arm_generation
```

```bash
ros2 topic info -v /roomba/cmd_vel_unstamped
```

PASS条件:

- `roomba-bridge-tcp`が1個。
- ARM topicのbridge subscriberが1個。
- command topicのbridge subscriberが1個。
- latency trace pathがtrial sessionディレクトリ内に設定される。
- 車輪停止、異音なし、再回転なし。

FAIL/中止条件: READY/STOP/Safe不成立、重複所有者、status `0x01`、timeout、通信断、物理動作。

## 5. Phase 2：両輪浮上・独立単発試験

各試行は必ず「STOP確認→個別承認→1回だけARM/Drive→STOP→再発進なし確認」で行う。
異常試行では再ARMしない。1つの承認を次の試行へ流用しない。

`ALLOW_NONZERO_DRIVE=True`への変更は、Phase 1が全項目PASSし、当該試行の承認文を
受け取った後に限る。変更後はPicoを再起動し、待受・車輪停止を再確認する。試行終了後は
ブリッジをSTOP-onlyで再確認し、`ALLOW_NONZERO_DRIVE=False`へ戻す。承認前、状態不明、
通信再接続後に以前のARM generationや候補を再利用してはならない。

### 共通記録

- trial ID、承認文、ARM generation
- Navigation期待Twist、期待左右輪速度
- bridge Drive ACK sequence/status/right/left
- host deadline/STOP begin/return
- Pico UART STOP完了
- 物理方向、物理停止、再発進の有無

既存`roomba-wheel-lift-test`を再利用する。各コマンドの
`ARM_GENERATION`は試行直前に生成した未使用の整数へ置換する。以下は承認後に1つずつ
提示・実行するためのテンプレートであり、この手順書を読むだけでは実行しない。

```bash
# 直進（既存mode名rotateを使うが、左右指定は+20/+20）
docker --context colima-create3 compose -f compose.yaml exec simulator bash -lc 'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/roomba-wheel-lift-test --ros-args -p mode:=rotate -p left_mm_s:=20.0 -p right_mm_s:=20.0 -p drive_duration_s:=0.4 -p allow_rotation:=true -p confirmation:=WHEELS_LIFTED -p arm_generation:=ARM_GENERATION'

# 左旋回
docker --context colima-create3 compose -f compose.yaml exec simulator bash -lc 'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/roomba-wheel-lift-test --ros-args -p mode:=rotate -p left_mm_s:=-20.0 -p right_mm_s:=20.0 -p drive_duration_s:=0.4 -p allow_rotation:=true -p confirmation:=WHEELS_LIFTED -p arm_generation:=ARM_GENERATION'

# 右旋回
docker --context colima-create3 compose -f compose.yaml exec simulator bash -lc 'source /opt/create3_ws/install/setup.bash; exec /usr/local/bin/roomba-wheel-lift-test --ros-args -p mode:=rotate -p left_mm_s:=20.0 -p right_mm_s:=-20.0 -p drive_duration_s:=0.4 -p allow_rotation:=true -p confirmation:=WHEELS_LIFTED -p arm_generation:=ARM_GENERATION'
```

実行場所は`/Users/imaichika/robot/simulation`。同じgenerationを再利用しない。
この単発publisherはNavigation本体ではなく、Phase 0で算出した左右輪候補を独立に物理確認するもの。

### 2A 直進

- 期待: `linear.x>0`, `angular.z=0`, left/right=`+20/+20`。
- PASS: 両輪が同じ前進方向、ACK `0x03`、400 ms以内にSTOP、再発進なし。
- FAIL: 片輪逆転、方向不一致、20 mm/s超過、STOP不成立。

### 2B 左旋回

- Navigation期待: `angular.z>0`。
- 期待輪符号: left=`-20`, right=`+20`。
- PASS: ログ符号に加え、機体上面から見て反時計回り（左）へ物理回転する。
- FAIL: ログだけ正しく物理方向が逆、片輪のみ、回転なし、STOP不成立。

### 2C 右旋回

- Navigation期待: `angular.z<0`。
- 期待輪符号: left=`+20`, right=`-20`。
- PASS: ログ符号に加え、機体上面から見て時計回り（右）へ物理回転する。
- FAIL条件は左旋回と対称。

### 2D STOP

- ゼロTwistだけを送る。ARMや非ゼロDriveは不要。
- PASS: STOP ACK `0x07`、要求0/0、Safe 2、物理停止、5秒間再発進なし。

400 msでは角度が小さいため、2B/2Cは通信・符号・物理方向の試験であり、角度精度試験ではない。

## 6. Phase 3：ArUcoによる前後比較

Phase 2がPASSし、固定カメラで試験前後のID 0が3連続検出できる場合だけ実施する。

1. 800x600、固定位置、marker ID 0、marker size 0.080 mを確認する。
2. 開始前に`/robot/pose_candidate`からx/y/yaw、header stampを記録する。
3. 有人単発を1回実施し、完全停止を確認する。
4. 停止後に新鮮な3連続姿勢を取得し、中央値を終了値とする。
5. `dx,dy,hypot(dx,dy),normalize(yaw_end-yaw_start)`を記録する。

姿勢欠損、frame不一致、0.5秒超の古い姿勢、map bounds外、明らかな外れ値はFAILではなく測定無効。
較正誤差・画素分解能より小さい差は「方向確認のみ」とし、移動量/角度PASSの証拠にしない。
エンコーダを同時取得していない限り、ArUco結果をエンコーダ実測と呼ばない。

## 7. Phase 4：閉ループ検証の準備

今回実装・実行しない。次段階では次を別々に検証する。

1. 目標方位へ1回だけ旋回。
2. STOP後にArUco姿勢を再取得。
3. 新しい角度誤差から新しい候補を計算。
4. 人間の新しい承認後だけ次のARM/単発を許可。
5. 誤差が許容内なら`approach`へ切替。
6. 距離減少と到達STOPを確認。

連続自動ARM、長時間走行、400 ms制限解除、古い目標による再発進は対象外である。

## 8. 全試験共通の終了処理

1. ゼロTwistとSTOPを確認。
2. Terminal AをCtrl+Cで終了し、trial sessionログ保存を確認。
3. 両輪停止・異音なし・5秒間再発進なしを目視確認。
4. Picoを停止し、`ALLOW_NONZERO_DRIVE=False`へ戻して保存・再起動確認。
5. STOP-only診断でACK `0x07`、0/0、Safe 2を再確認。
6. Pico USBを抜き、LED消灯とRoomba停止を確認。

## 9. 非走行検証結果

- Phase 0行列: PASS（実機、ARM、Drive未使用）。
- Navigation関連重点回帰: 90/90 PASS。
- 全回帰: 405/405 PASS、除外なし。
- 実機Phase 1--3: 未実施、個別承認待ち。
