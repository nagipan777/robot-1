# ROS 2 Jazzy + Gazebo Harmonic + iRobot Create 3

Apple Silicon Mac上のColima専用VM（`create3`）でUbuntu 24.04 ARM64コンテナを動かします。ROS 2、Gazebo、Create 3はコンテナ内にインストールします。Mac側の `.venv`、カメラコード、TACO学習データは共有・変更しません。

## 起動

```bash
cd /Users/imaichika/robot
./simulation/sim.sh up
```

ブラウザで http://localhost:6080/vnc.html?autoconnect=true&resize=scale を開きます。初回はイメージのビルドに時間がかかります。

Macの別ターミナルでWebカメラとYOLOを起動します。

```bash
source .venv/bin/activate
python vision.py
```

検出結果は自動的にシミュレーターへ送られ、ROS 2の `/webcam/detections` にJSONとして配信されます。確認するには次を実行します。

```bash
./simulation/sim.sh shell
ros2 topic echo /webcam/detections
```

シミュレーターへ送信せずカメラだけ使う場合は `python vision.py --simulator ''`、映像確認だけの場合は `python vision.py --preview` を使います。

## ごみ方向への旋回制御（未校正・無効）

`vision_turn_controller` は `/webcam/detections` からクラス名が `litter` の検出だけを選び、シミュレーターでは `/odom` のロボット姿勢を購読します。画像座標をodom平面へ変換した後、旋回指令を `/cmd_vel_unstamped`（`geometry_msgs/msg/Twist`）へ送る構成です。シミュレーター内のリレーが、これをCreate 3用の `/cmd_vel`（`TwistStamped`）へ変換します。

画像座標とodom座標はまだ校正されていないため、[vision_controller.yaml](vision_controller.yaml) の `control_enabled` と `calibration_ready` は両方とも `false` です。この状態では制御ノードは速度指令を出しませんが、座標変換結果は確認できます。

```bash
# 変換後の目標点
ros2 topic echo /litter/target

# 到達判定。距離15cm以内なら true
ros2 topic echo /litter/reached

# 変換ログ
./simulation/sim.sh logs
```

RVizではFixed Frameを `odom` にして、Marker表示へ `/litter/target_marker` を指定すると、変換後の目標がオレンジ色の球で表示されます。検出が1秒以上途切れるとMarkerを消し、制御中であれば停止します。

表示経路だけを先に確認する場合、既知の床座標 `odom (1.5, 0.0)` に緑色のテストMarkerを配信します。

```bash
./simulation/sim.sh test-marker
```

VNC内でGazeboとは別のRVizウィンドウへ切り替えて確認します。確認後は `Ctrl+C` で停止します。これはRViz Markerであり、Gazebo内へ物理的なゴミモデルを生成する機能ではありません。

### 床面座標の校正

現在のワールドは20m四方の床、原点付近のロボット、`(0.157, 0)` のドックで構成されています。これらを避けた仮想作業エリアを `x=0.5..2.5m, y=-1.0..1.0m` とします。

`vision.py` を `q` で終了してカメラを解放した後、対話式校正を起動します。

```bash
source .venv/bin/activate
python calibrate_floor.py --camera 0
```

校正画面は `vision.py` と同じ既定YOLOモデルを読み込み、`litter` の枠・中心画素を紫色で重ねます。4点クリック後は、その中心画素から変換したodom座標も右側に表示されます。YOLOを使わず校正だけ行う場合は `--no-yolo` を指定します。

画像上の作業エリアを、画面表示に従って次の順にクリックします。

1. 左上 → odom `(2.5, 1.0)`
2. 右上 → odom `(2.5, -1.0)`
3. 右下 → odom `(0.5, -1.0)`
4. 左下 → odom `(0.5, 1.0)`

4点後は四隅の再投影誤差と中央点 `odom (1.5, 0.0)` が画面・ターミナルに表示されます。`S` で [vision_controller.yaml](vision_controller.yaml) へホモグラフィと画像解像度を保存、`R` でやり直し、`Q` またはEscで中止します。別カメラは `--camera 1` のように指定します。運用時の `vision.py` の解像度が校正時と違う場合、制御ノードは座標を破棄して停止します。

保存時も `control_enabled: false` と `calibration_ready: false` は維持されます。さらに制御ノードは単位行列を校正済みとして受け付けません。保存後に `./simulation/sim.sh up` で反映し、`/litter/target` とMarkerが実際の床面位置に一致することを確認するまで安全フラグを変更しないでください。

シミュレーターでの吸引完了は物理的な吸引ではなく、`/litter/reached` が `true` になる到達判定として扱います。

実機のArUco姿勢へ切り替える際は、同じ設定で `pose_source: pose_stamped` とし、`pose_topic` に校正済み世界座標系の `geometry_msgs/msg/PoseStamped` トピックを指定します。制御計算側の変更は不要です。

ローカルの床とCreate 3・充電ドックを表示します。外部のGazebo Fuelモデルは不要です。MacのGPUはコンテナから使用せず、ソフトウェア描画を使います。8GB RAMのMac用にVMを4GB・2コアにし、RVizは自動起動しません。

## ROS 2と操作

```bash
./simulation/sim.sh shell
```

コンテナ内で：

```bash
ros2 topic list
ros2 action list
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}'
```

停止中ならGazebo画面の再生ボタンを押します。別のMacターミナルでキーボード操作を開始できます。

```bash
./simulation/sim.sh teleop
```

操作方法は表示されるキー案内に従ってください。Create 3のJazzy版は `TwistStamped` を使うため、`stamped:=true` を設定しています。

## 停止・ログ

```bash
./simulation/sim.sh status
./simulation/sim.sh logs
./simulation/sim.sh down  # コンテナ停止・削除（イメージは保持）
./simulation/sim.sh stop  # コンテナと専用VMを停止し、メモリを解放
```

Dockerの既定コンテキストは変更せず、常に `colima-create3` を指定します。VMにはMacのフォルダをマウントしていません。ブラウザ用ポート6080はMacのlocalhostだけに公開します。ROSの自動探索はコンテナ内のlocalhostに限定し、実機には接続しません。

## 構成と出典

- ROS 2 Jazzy / Ubuntu 24.04: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- Gazebo Harmonic / ROS連携: https://gazebosim.org/docs/harmonic/ros_installation/
- Create 3公式: https://github.com/iRobotEducation/create3_sim/tree/jazzy
- メッセージ定義: https://github.com/iRobotEducation/irobot_create_msgs
- Create 3 source commit: `6c305843afcfe094bf8eb2a0c6997f531e526b9d`
- Messages source commit: `b4986ef5dc2e07d8aa7471f29c56b617473352c5`

公式ソースはこのフォルダの `create3_sim/` と `irobot_create_msgs/` に取得済みです。ライセンスは各リポジトリを参照してください。

このMac向けの変更として、Gazebo起動に `-r` を付けて制御ノードの起動前から時間を進め、制御更新周期を100Hzにしています。初回のシェーダー生成には時間がかかることがあります。

ソースやDockerfileを変更して再ビルドする場合：

```bash
docker --context colima-create3 compose -f simulation/compose.yaml build
./simulation/sim.sh up
```
