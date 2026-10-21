# PAI最終課題：ROS 2動画撮影と実機統合準備

この文書は、既存のCreate 3シミュレーションを維持したまま、ROS 2を使った認識・判断・制御を動画で示すための手順です。`record-status`は購読専用であり、速度や設定を変更しません。

## 1. 変更前ベースライン（2026-10-12）

- `.venv/bin/python -m unittest discover -v`: **38件成功**
- `simulation/sim.sh`と`simulation/start-simulation.sh`: `bash -n`成功
- Create 3起動経路: `sim.sh` → Docker Compose → Supervisor → `create3_gz.launch.py`
- MacからDockerへの検出転送: `127.0.0.1:8765`へのHTTP POST
- 制御経路: `/cmd_vel_unstamped` → Create 3用relay → `/cmd_vel`
- 稼働中コンテナで`vision_turn_controller`、`map_frame_adapter`、`unstamped_cmd_relay`と主要トピック型を読み取り確認済み
- 既存worktreeは変更済み・未追跡ファイルを含む。リセット、削除、移動は実施していない。

重要な未追跡ファイルには、`ARCHITECTURE.md`、`roomba_driver.py`、`odometry.py`、`differential_drive.py`、`navigation.py`、`requirements.txt`、`tests/`、`simulation/map_frame_adapter.py`、`simulation/roomba_dry_run_adapter.py`が含まれていた。

## 2. 撮影前の安全確認

1. 今回の動画はCreate 3シミュレーションだけで撮影し、Roomba 680を接続しない。
2. `control_enabled`と`calibration_ready`は起動時に`false`であることを確認する。
3. 最初は`turn_only`で旋回方向を確認する。
4. `approach`へ切り替える前にGazebo内の直線経路に障害物がないことを確認する。
5. 停止用ターミナルを常に開いておく。

## 3. 推奨画面配置

- 左半分: VNCブラウザ。`presentation-layout`でGazeboとRVizを自動配置する。
- 右上: MacのYOLO映像。
- 右下: `record-status`のROS 2トピック集約表示。
- 必要に応じて短時間だけ別ターミナルで`ros2 node list`と`ros2 topic info`を表示する。

画面録画にはmacOSの画面収録を使い、シリアル番号、個人情報、不要な別ウィンドウが映らないことを事前に確認する。

## 4. 起動

ターミナルAでシミュレーションを起動する。

```bash
cd /path/to/robot-1
./simulation/sim.sh up
./simulation/sim.sh status
./simulation/sim.sh presentation-layout
```

ブラウザで次を開く。

```text
http://localhost:6080/vnc.html?autoconnect=true&resize=scale
```

Gazeboには撮影用の室内、走行レーン、照明、スケール確認用の家具が表示される。RVizにはシアンの走行軌跡・進行方向、オレンジの目標線、到達時に緑になる目標、速度付き状態表示が重なる。これらは表示専用で制御アルゴリズムを変更しない。

ターミナルBで、読み取り専用のROS 2集約表示を起動する。

```bash
./simulation/sim.sh record-status
```

ターミナルCでYOLOを起動する。`--quiet`はフレームごとの検出ログだけを抑制し、映像とエラー表示は維持する。

```bash
source .venv/bin/activate
python vision.py --quiet
```

## 5. ROS 2使用の証明

コンテナシェルでノードと型を表示する。

```bash
./simulation/sim.sh ros-status
```

期待する型:

```text
/webcam/detections  std_msgs/msg/String
/litter/target      geometry_msgs/msg/PointStamped
/cmd_vel_unstamped  geometry_msgs/msg/Twist
/robot/pose         geometry_msgs/msg/PoseStamped
```

集約表示では、次の1行が1秒ごとに更新される。

```text
ROS2 FLOW | detections=... | target=... | pose=... | command=... | reached=...
```

生メッセージを撮影する場合は、各コマンドを短時間だけ実行する。

```bash
ros2 topic echo /webcam/detections
ros2 topic echo /litter/target
ros2 topic echo /cmd_vel_unstamped
```

## 6. 制御を段階的に有効化

最初に校正済みであることを目視確認し、旋回だけを有効化する。

```bash
./simulation/sim.sh control-turn
```

動画では、ゴミを左右へ動かし、次を同時に示す。

- YOLOの検出枠
- `/litter/target`の変化
- `/cmd_vel_unstamped.angular.z`の符号変化
- RViz Markerの移動
- Gazebo上のCreate 3の旋回
- ゴミを画面外へ移したときのゼロ速度

旋回を確認した後だけ、低速接近へ切り替える。

```bash
./simulation/sim.sh control-approach
```

目標へ接近し、`reached=true`とゼロ速度を撮影する。

## 7. 停止と終了

通常停止は、先に制御を無効化してゼロ速度を確認する。

```bash
./simulation/sim.sh control-off
```

その後、YOLOと`record-status`を`Ctrl+C`で終了し、シミュレーションを停止する。

```bash
./simulation/sim.sh down
```

## 8. 動画の受け入れチェックリスト

- [ ] YOLO映像に`litter`の検出枠が映っている
- [ ] `webcam_bridge`と`vision_turn_controller`が`ros2 node list`に表示される
- [ ] `/webcam/detections`の型と値を示した
- [ ] `/litter/target`の型と座標変化を示した
- [ ] `/cmd_vel_unstamped`の型と速度変化を示した
- [ ] RVizに目標Markerが表示された
- [ ] RVizに軌跡、進行方向、目標線、速度状態が表示された
- [ ] Gazeboで旋回、前進、停止を示した
- [ ] 検出消失または制御無効化による停止を示した
- [ ] Roomba 680実機には接続していない

## 9. 最小リファクタリング方針

- `vision_turn_controller.py`を共通Navigationとして維持する。
- `/cmd_vel_unstamped`をCreate 3と将来のRoomba adapterの共通境界にする。
- `unstamped_cmd_relay.py`はCreate 3専用のまま維持する。
- `roomba_driver.py`、`odometry.py`、`differential_drive.py`はROS 2非依存を維持する。
- `goal_math.py`は互換importだけを残し、計算実装を`navigation.py`へ統一した。
- `go_to_goal.py`と`test_navigation.py`は同一テストであり、提出前整理の対象とする。
- `roomba_dry_run_adapter.py`の速度変換は`differential_drive.py`と重複する。ただし現在のDocker build contextからルートモジュールをimportできないため、動画撮影前には変更しない。

この段階では、動作中のシミュレーションを優先し、上記重複ファイルの削除・移動・リネームは行わない。
