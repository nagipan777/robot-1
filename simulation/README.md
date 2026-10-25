# ROS 2 Jazzy + Gazebo Harmonic + iRobot Create 3

Apple Silicon Mac上のColima専用VM（`create3`）でUbuntu 24.04 ARM64コンテナを動かします。ROS 2、Gazebo、Create 3はコンテナ内にインストールします。Mac側の `.venv`、カメラコード、TACO学習データは共有・変更しません。

## 起動

```bash
cd /Users/imaichika/robot
./simulation/sim.sh up
```

固定RGBカメラ付きの別ワールドを起動する場合だけ、次を使います。通常の`up`は従来どおり`room.sdf`を使用し、カメラブリッジや画像ビューアーを起動しません。

```bash
./simulation/sim.sh camera-up
./simulation/sim.sh camera-status
```

`camera-up`ではVNC内に`rqt_image_view`が自動的に開き、`/sim/fixed_camera/image_raw`を表示します。画像は`640x480`、10 HzのRGBです。カメラは床上4.5 m、水平画角1.45 radで真下を向き、6 m × 4 mの床面全体を覆います。

同時にMJPEGをMacのlocalhostへ配信します。ポート8766は`127.0.0.1`だけに公開されます。

```bash
curl http://127.0.0.1:8766/health
source .venv/bin/activate
python vision.py --video-url http://127.0.0.1:8766/stream.mjpg
```

この入力でも実Webカメラと同じ検出JSONを`http://127.0.0.1:8765/detections`へPOSTします。実Webカメラへ戻す場合は`python vision.py --camera 0`を実行します。切り替えはMac側の`vision.py`だけで完結し、ROS 2ノードの再構築は不要です。通常の`sim.sh up`ではMJPEGノードを含むカメラ関連プロセスは起動しません。

## 仮想ゴミと仮想カメラ校正

カメラワールドだけで使用するvisual-onlyの空き缶モデルを操作できます。

```bash
./simulation/sim.sh litter-add can 1.5 -0.7
./simulation/sim.sh litter-move paper 1.0 1.0
./simulation/sim.sh litter-add bottle 1.5 -0.7
./simulation/sim.sh litter-remove
```

`can`、`paper`、`bottle`を選択できます。種類を省略した従来形式は`can`のままです。

既存モデルのYOLO評価は次で行います。未検出や背景への偽陽性もそのままJSONと画像へ保存し、自動再学習は行いません。

```bash
source .venv/bin/activate
python evaluate_sim_litter.py
```

同一条件の背景画像と3種類のモデルを比較する再現可能な実験は次で実行します。

```bash
python compare_sim_litter.py
```

既存重み、confidence閾値0.5、ホモグラフィを固定し、差分画像から正解領域と画像上のピクセル数を測定します。検出枠、confidence、IoU、背景の偽陽性と未検出の区別は`artifacts/sim_litter_comparison/results.{json,md}`へ保存されます。

仮想カメラ校正は床面とほぼ同じ高さのマーカーを既知座標へ配置し、4校正点と3つの未使用検証点を測ります。ソフトウェア描画が低速な場合があるため、完了まで数分かかることがあります。

```bash
python collect_sim_camera_points.py
python calibrate_sim_camera.py --max-error 0.05
```

検証点の最大誤差が閾値を超えた場合、設定は更新されません。合格時も仮想カメラ専用の`vision_controller.sim_camera.yaml`だけを更新し、両安全フラグはfalseのままです。`camera-up`はこの専用設定を読み、通常の`up`は従来の`vision_controller.yaml`を読みます。

ブラウザで http://localhost:6080/vnc.html?autoconnect=true&resize=scale を開きます。初回はイメージのビルドに時間がかかります。

提出動画ではGazeboとRVizを撮影用に並べます。

```bash
./simulation/sim.sh presentation-layout
```

Gazeboは室内風の撮影用ワールドを表示します。RVizの`/pai/presentation_markers`には走行軌跡、進行方向、目標までの線、到達状態、現在の速度を表示します。`presentation_visualizer.py`は読み取り専用で、車輪制御トピックへpublishしません。

Macの別ターミナルでWebカメラとYOLOを起動します。

```bash
source .venv/bin/activate
python vision.py --camera 0
```

検出結果は自動的にシミュレーターへ送られ、ROS 2の `/webcam/detections` にJSONとして配信されます。確認するには次を実行します。

```bash
./simulation/sim.sh shell
ros2 topic echo /webcam/detections
```

シミュレーターへ送信せずカメラだけ使う場合は `python vision.py --simulator ''`、映像確認だけの場合は `python vision.py --preview` を使います。

## ごみ方向への旋回制御（未校正・無効）

`vision_turn_controller`は`/webcam/detections`からクラス名が`litter`の検出だけを選び、画像座標をまず`odom`床面座標へ変換します。`map_frame_adapter`がごみ位置と`/odom`姿勢の両方へ同じ`map←odom`変換を数値適用し、`map`座標系の`/litter/target`と`/robot/pose`を生成します。制御ノードはこの2入力から`/cmd_vel_unstamped`（`geometry_msgs/msg/Twist`）へ速度指令を送り、シミュレーター内のリレーがCreate 3用の`/cmd_vel`（`TwistStamped`）へ変換します。

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
python calibrate_floor.py --camera 0 --no-yolo
```

校正画面は `vision.py` と同じ既定YOLOモデルを読み込み、`litter` の枠・中心画素を紫色で重ねます。4点クリック後は、その中心画素から変換したodom座標も右側に表示されます。YOLOを使わず校正だけ行う場合は `--no-yolo` を指定します。

画像上の作業エリアを、画面表示に従って次の順にクリックします。

1. 左上 → odom `(2.5, 1.0)`
2. 右上 → odom `(2.5, -1.0)`
3. 右下 → odom `(0.5, -1.0)`
4. 左下 → odom `(0.5, 1.0)`

4点後、4点とは独立に実測した中央点 `odom (1.5, 0.0)` を5回目にクリックします。独立検証点誤差が画面・ターミナルに表示された後、`S` で [vision_controller.yaml](vision_controller.yaml) へホモグラフィと画像解像度を保存、`R` でやり直し、`Q` またはEscで中止します。別カメラは `--camera 1` のように指定します。運用時の `vision.py` の解像度が校正時と違う場合、制御ノードは座標を破棄して停止します。

保存時も `control_enabled: false` と `calibration_ready: false` は維持されます。さらに制御ノードは単位行列を校正済みとして受け付けません。保存後に `./simulation/sim.sh up` で反映し、`/litter/target` とMarkerが実際の床面位置に一致することを確認するまで安全フラグを変更しないでください。

シミュレーターでの吸引完了は物理的な吸引ではなく、`/litter/reached` が `true` になる到達判定として扱います。

実機のArUco姿勢へ切り替える際は、ArUcoノードが`map`座標系の`/robot/pose`（`geometry_msgs/msg/PoseStamped`）を生成します。シミュレーターの`map_frame_adapter`と同じ出力契約なので、制御計算側の変更は不要です。

ローカルの室内ワールドとCreate 3・充電ドックを表示します。外部のGazebo Fuelモデルは不要です。MacのGPUはコンテナから使用せず、ソフトウェア描画を使います。8GB RAMのMac用にVMを4GB・2コアにし、RVizも自動起動します。

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

### 既知の床面座標でNavigationを検証する

第4段階Aのテスト経路は次のとおりです。

```text
/litter/target_odom (PointStamped, odom; テスト専用)
  -> map_frame_adapter
  -> /litter/target (PointStamped, map)
  -> vision_turn_controller + /robot/pose (PoseStamped, map)
  -> /cmd_vel_unstamped (Twist)
  -> unstamped_cmd_relay
  -> /cmd_vel (TwistStamped)
  -> Create3
```

`/litter/target`へ直接テストPublisherを追加すると通常の`map_frame_adapter`と競合するため、入力は既存の`/litter/target_odom`に限定しています。YOLOおよび`/webcam/detections`は使用しません。

```bash
./simulation/sim.sh nav-test-up
./simulation/sim.sh known-target-test
```

`nav-test-up`は`room`ワールドと検証済みシミュレーション用設定を使う明示的な試験起動です。通常のドッキング初期配置で起きるバンパー接触を分離するため、この起動だけはCreate3を世界座標`(0, -0.75)`へ置き、ドックを生成しません。通常の`up`と`camera-up`は従来の原点・ドックありです。制御は無効のまま開始し、テストランナーが校正誤差と速度上限を確認してから一時的に有効化します。旋回、前進、到達停止、目標入力を止めた後の1秒タイムアウト停止を検証し、終了時には成否にかかわらず制御・校正フラグを無効化します。

ログはホストの`artifacts/known_target_test/result.json`に保存されます。異常終了時のfinally停止は、ワールドを初期化して次の明示的な試験で確認できます。

```bash
./simulation/sim.sh nav-test-up
./simulation/sim.sh known-target-abort-test  # 非0終了が期待値
```

通常の`./simulation/sim.sh up`と`camera-up`には、このテストPublisherや自動的な制御有効化は追加されません。

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
