# robot-1 チュートリアル

> 2026-10-17のコード・ログ照合と整理方針は [ARCHITECTURE.md](ARCHITECTURE.md) を参照してください。mainと開発ブランチ、実装と実機検証の差を明記しています。


PAI最終課題のROS 2動画を撮影する場合は、ベースライン、推奨画面配置、トピック確認、段階的な制御有効化、停止手順をまとめた[PAI_SUBMISSION.md](PAI_SUBMISSION.md)を先に確認してください。

固定Webカメラでごみを検出し、カメラ画像上の座標をGazeboの床面座標へ変換して、Create 3シミュレーター上で確認するプロジェクトです。

このチュートリアルでは、次の順番で安全に動作確認します。

1. シミュレーターを起動する
2. 既知の床座標にテスト用Markerを表示する
3. Webカメラ画像と`/odom`座標を校正する
4. YOLOの検出結果をROS 2へ送る
5. 変換後のごみ座標をRVizとログで確認する

> [!IMPORTANT]
> 自動走行は初期状態で無効です。座標対応を確認するまで、`simulation/vision_controller.yaml`の`control_enabled`と`calibration_ready`を`false`のままにしてください。検出が途切れた場合や古いデータを受け取った場合も、ロボットは停止する設計です。

## 1. 構成

固定WebカメラとYOLOはMac側、ROS 2・Gazebo・Create 3はColima/Docker側で動きます。

```text
Webカメラ → vision.py → HTTP → /webcam/detections
                                      ↓
                         /litter/target_odom ─┐
                                             ├→ map_frame_adapter
                                      /odom ─┘        │
                              /robot/pose (map) ──────┤
                            /litter/target (map) ─────┘
                                             ↓
                                vision_turn_controller
                                             ↓
                             /cmd_vel_unstamped
                                      ↓
                        TwistStamped relay → Create 3 / Gazebo
```

プロセス境界、全トピック、座標系、安全条件、シミュレーターと実機の差し替え方は[ARCHITECTURE.md](ARCHITECTURE.md)にまとめています。

現在のごみ表示は`visualization_msgs/msg/Marker`を使った**RViz上の可視化**です。Gazebo内に物理的なごみモデルを生成する機能ではありません。シミュレーターでの「吸引」は、目標から15 cm以内に入ったとき`/litter/reached`が`true`になる到達判定として扱います。

## 2. 必要なもの

- Apple Silicon Mac
- 固定できるWebカメラ
- Python 3
- Docker CLIとColima
- 4 GB以上の空きメモリ
- 初回セットアップ時のインターネット接続

コマンドが使えることを確認します。

```bash
python3 --version
docker --version
colima version
```

## 3. リポジトリとPython環境を準備する

```bash
git clone https://github.com/nagipan777/robot-1.git
cd robot-1

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install ultralytics opencv-python numpy pillow
```

学習データと学習済み重みは容量が大きいため、GitHubには含まれていません。YOLOを使うには、学習済み重みを次の場所へ置いてください。

```text
runs/detect/taco/weights/best.pt
```

重みをまだ作っていない場合は、TACOデータセットから学習できます。

```bash
source .venv/bin/activate
python prepare_taco.py
yolo detect train model=yolo26n.pt data=taco_dataset/data.yaml epochs=40 imgsz=640 device=mps name=taco
```

学習の詳細は[TACO.md](TACO.md)を参照してください。

## 4. シミュレーターを起動する

リポジトリのルートで実行します。

```bash
./simulation/sim.sh up
```

初回はColima VMとDockerイメージを準備するため時間がかかります。起動後、ブラウザで次を開きます。

<http://localhost:6080/vnc.html?autoconnect=true&resize=scale>

Gazeboの固定RGBカメラを確認する場合は、通常ワールドとは別のカメラ付きワールドを起動します。

```bash
./simulation/sim.sh camera-up
./simulation/sim.sh camera-status
```

VNC内の`rqt_image_view`へ`/sim/fixed_camera/image_raw`が自動表示されます。通常の`./simulation/sim.sh up`ではカメラ関連プロセスは起動しません。
`camera-up`のときだけ、同じ画像をMacのlocalhostへMJPEG配信します。

```bash
# 配信状態の確認
curl http://127.0.0.1:8766/health

# 仮想固定カメラをYOLO入力にして、従来と同じ検出JSONを送信
source .venv/bin/activate
python vision.py \
  --video-url http://127.0.0.1:8766/stream.mjpg \
  --weights datasets/PAI_output/best.pt \
  --confidence 0.25 \
  --http-timeout 2.0
```

実Webカメラへ戻す場合は`python vision.py --camera 0`を使います。入力切り替えでROS 2側を再ビルド・再起動する必要はありません。MJPEGポート8766はDockerからMacの`127.0.0.1`だけへ公開され、LANには公開されません。

8 GB MacでGazeboとYOLOを同時実行するとMJPEGのフレーム間隔が長くなる場合があります。
自律走行試験では、同じ固定カメラの安定したスナップショット入力を使用します。

```bash
./simulation/sim.sh vision-integration-up
./simulation/sim.sh litter-add can 0.5 0.5

.venv/bin/python vision.py \
  --video-url http://127.0.0.1:8766/snapshot.jpg \
  --weights datasets/PAI_output/best.pt \
  --confidence 0.25 \
  --http-timeout 6.0 \
  --quiet

# 別ターミナル。安全確認後だけ一時的に制御を有効化し、終了時に無効へ戻す。
./simulation/sim.sh yolo-navigation-test
```

結果は`artifacts/yolo_navigation_test/result.json`へ保存されます。通常起動の
`control_enabled=false`と`calibration_ready=false`は変更しません。

`datasets/PAI_output/best.pt`はTACO+Gazebo混合学習モデルです。Gazebo固定testでは
`confidence=0.25`で背景偽陽性0件/枚でした。実写TACO testでは元モデルより性能が低下したため、
実Webカメラでは従来モデルを既定のまま維持しています。明示する場合は
`--weights runs/detect/taco/weights/best.pt --confidence 0.5`を指定してください。
評価の再実行は`.venv/bin/python validate_pai_output.py`で行い、結果は
`artifacts/pai_output_validation/`へ保存されます。

### 仮想ゴミと仮想カメラ校正

`camera-up`の起動後、視覚確認用の仮想ゴミを配置・移動・削除できます。モデルはカメラ検証専用のvisual-onlyモデルで、Create3の衝突判定には影響しません。

```bash
./simulation/sim.sh litter-add can 1.5 -0.7
./simulation/sim.sh litter-move paper 1.0 1.0
./simulation/sim.sh litter-add bottle 1.5 -0.7
./simulation/sim.sh litter-remove
```

種類を省略した従来形式は空き缶として扱います。比較対象は`can`、`paper`、`bottle`です。

既存YOLOモデルを再学習せず評価し、生画像、注釈画像、検出JSONを保存します。

```bash
source .venv/bin/activate
python evaluate_sim_litter.py
```

3種類を背景画像と同一条件で比較し、検出枠、confidence、正解領域とのIoUを保存する場合は次を実行します。既存の重み、confidence閾値0.5、校正値は変更しません。

```bash
python compare_sim_litter.py
```

結果は`artifacts/sim_litter_comparison/results.{json,md}`と種類別の画像へ保存されます。今回の固定条件では、空き缶と紙の検出は正解領域とのIoUが0.02未満で背景由来の偽陽性、ボトルは未検出でした。このため、3種類とも再学習なしで認識可能とは判定していません。

仮想カメラの校正は、床面上のマゼンタ色マーカーを既知位置へ順番に配置して画素中心を測ります。4点を行列計算、別の3点を誤差検証に使用します。

```bash
python collect_sim_camera_points.py
python calibrate_sim_camera.py
```

結果は`simulation/vision_controller.sim_camera.yaml`だけへ保存されます。実Webカメラ用の`simulation/vision_controller.yaml`は変更しません。校正値を書き込んだ後も`control_enabled=false`と`calibration_ready=false`を強制的に維持します。

提出動画用にGazeboを左、RVizを右へ自動配置します。新しい室内ワールド、走行軌跡、進行方向、目標線、速度と状態の文字表示も自動で有効になります。

```bash
./simulation/sim.sh presentation-layout
```

表示の意味は、シアンが軌跡と進行方向、オレンジが目標までの線と未到達目標、緑が到達済み目標です。表示ノードは購読とMarker配信だけを行い、速度指令は生成しません。

VNC内ではGazeboとRVizが別ウィンドウで動きます。必要に応じて`Alt+Tab`で切り替えてください。状態とログはMac側の別ターミナルから確認できます。

```bash
./simulation/sim.sh status
./simulation/sim.sh logs
```

## 5. 既知の床座標をRVizで確認する

YOLOと校正をつなぐ前に、Markerの表示経路だけを確認します。

```bash
./simulation/sim.sh test-marker
```

RVizで次を確認してください。

- Fixed Frame: `odom`
- Markerトピック: `/litter/target_marker`
- 表示位置: `odom (1.5, 0.0)`
- 表示内容: 緑色の球

確認できたら、`test-marker`を実行したターミナルで`Ctrl+C`を押します。Gazebo画面に球が出ないのは正常です。このMarkerはRViz専用です。

## 6. 固定Webカメラと床面を校正する

校正中は`vision.py`を終了して、カメラを解放してください。同じWebカメラを同時に2つのプログラムから開くことはできません。

```bash
source .venv/bin/activate
python calibrate_floor.py --camera 0
```

画面に表示される案内に従い、固定カメラ画像上の作業エリアを次の順にクリックします。

| 順番 | 画像上の位置 | 対応する`odom`座標 |
|---:|---|---:|
| 1 | 左上 | `(2.5, 1.0)` |
| 2 | 右上 | `(2.5, -1.0)` |
| 3 | 右下 | `(0.5, -1.0)` |
| 4 | 左下 | `(0.5, 1.0)` |

この範囲は、原点付近のロボットと`(0.157, 0)`付近のドックを避けた仮想作業エリアです。

4点をクリックすると、四隅の再投影結果と中央点`odom (1.5, 0.0)`が画面とターミナルに表示されます。

- `S`: ホモグラフィと画像解像度を保存
- `R`: 最初からやり直す
- `Q`または`Esc`: 保存せず終了

別のカメラを使う場合は`--camera 1`のように番号を変更します。YOLO表示なしで校正する場合は`--no-yolo`を追加します。

```bash
python calibrate_floor.py --camera 1
python calibrate_floor.py --camera 0 --no-yolo
```

保存先は`simulation/vision_controller.yaml`です。単位行列は有効な校正として扱われません。また、保存後も安全フラグは自動的に有効になりません。

保存した設定をコンテナへ反映するため、シミュレーターを再起動します。

```bash
./simulation/sim.sh down
./simulation/sim.sh up
```

## 7. YOLO検出をROS 2へ送る

Mac側の別ターミナルで実行します。

```bash
cd /path/to/robot-1
source .venv/bin/activate
python vision.py --camera 0
```

映像ウィンドウも表示する場合は次を使います。

```bash
python vision.py --preview
```

`vision.py`は既定でシミュレーターの`http://127.0.0.1:8765/detections`へ検出JSONを送ります。送信せずカメラとYOLOだけを確認する場合は、空の送信先を指定します。

```bash
python vision.py --simulator '' --preview
```

カメラ付きワールドの映像を使う場合は、`camera-up`後に入力URLだけを変更します。検出JSONの形式と送信先は実Webカメラの場合と同じです。

```bash
python vision.py --video-url http://127.0.0.1:8766/stream.mjpg
```

## 8. ROS 2トピックと変換座標を確認する

シミュレーターのコンテナへ入ります。

```bash
./simulation/sim.sh shell
```

まず、検出JSONが届いていることを確認します。

```bash
ros2 topic info /webcam/detections
ros2 topic echo /webcam/detections
```

続いて、変換後の目標点、Marker、到達判定を確認します。

```bash
ros2 topic echo /litter/target
ros2 topic info /litter/target_marker
ros2 topic echo /litter/reached
```

RVizでは`/litter/target_marker`にオレンジ色の球が表示されます。YOLOのごみ中心を動かしたとき、Markerが校正した床面内の対応位置へ移動することを確認してください。変換ログはMac側からも確認できます。

```bash
./simulation/sim.sh logs
```

校正時と運用時で画像解像度が異なる場合、変換結果は安全のため破棄されます。カメラの位置・向き・高さ・画角を変えた場合も、必ず再校正してください。

## 9. シミュレーターで目標追従を確認する

追従制御は`map`座標系の`/litter/target`と`/robot/pose`を購読し、`/cmd_vel_unstamped`へ速度指令を送ります。最初に旋回だけを確認し、問題がなければ低速前進へ切り替えます。実機では実行しないでください。

最初のターミナルでYOLOを動かし、障害物のない床面にごみを置きます。

```bash
source .venv/bin/activate
python vision.py --quiet
```

別のターミナルでROS 2ノード、主要トピック、現在の安全パラメータを確認します。

```bash
./simulation/sim.sh ros-status
```

校正結果が正しいことを確認した後、旋回だけを明示的に有効化します。このコマンドは最初に制御を無効化してから、`calibration_ready=true`、`turn_only`、`control_enabled=true`の順に反映します。

```bash
./simulation/sim.sh control-turn
```

### 9.1 旋回だけを確認する

`turn_only`では`linear.x`を常に0に保ち、目標方向への旋回だけを行います。別のコンテナシェルで指令を確認できます。

```bash
ros2 topic echo /cmd_vel_unstamped
```

次を確認してください。

- `linear.x`が常に`0.0`
- ごみを左右へ動かすと`angular.z`の符号が変わる
- ロボットが目標方向を向くと`angular.z`が`0.0`になる
- ごみを画面から外すと、1秒以内にゼロ速度になる

### 9.2 障害物のない範囲で低速前進する

旋回確認後、ロボットから目標までの直線上にドックや障害物がないことをGazeboで確認します。その後、制御モードだけを切り替えます。

```bash
./simulation/sim.sh control-approach
```

`approach`では、目標との向きのずれが0.15 rad以内のときだけ、最大0.08 m/sで前進します。向きが大きくずれた場合は前進せず、旋回を優先します。目標から15 cm以内へ入ると停止し、次が`true`になります。

```bash
ros2 topic echo /litter/reached
```

### 9.3 停止する

通常停止では、別のコンテナシェルで先に速度指令を監視します。

```bash
ros2 topic echo /cmd_vel_unstamped
```

その状態で、制御を実行しているシェルから無効化します。ノードがゼロ速度を送信したことを監視側で確認し、`Ctrl+C`で監視を終了します。

```bash
./simulation/sim.sh control-off
```

`control-off`は制御を無効化した後、ゼロ速度を1回配信します。緊急時にコンテナ内から直接ゼロ速度を送る場合は次を使えます。

```bash
ros2 topic pub --once /cmd_vel_unstamped geometry_msgs/msg/Twist '{}'
```

`/litter/target`または`/robot/pose`が1秒より古くなった場合、検出が消えた場合、目標へ到達した場合、ノード終了時にもゼロ速度を送ります。

## 10. ロボットを手動で動かす

自動走行を有効にせず、キーボード操作で座標と到達判定を確認できます。

```bash
./simulation/sim.sh teleop
```

操作方法はターミナルに表示されます。ごみ目標へ15 cm以内まで近づくと、`/litter/reached`が`true`になります。

## 11. 終了する

カメラ画面では`q`を押して終了します。シミュレーターは用途に応じて停止します。

```bash
./simulation/sim.sh down  # コンテナを停止・削除し、イメージは保持
./simulation/sim.sh stop  # コンテナと専用Colima VMを停止
```

## トラブルシューティング

### `/webcam/detections`がpublishされていない

次の順に確認します。

1. `./simulation/sim.sh up`を先に実行したか
2. `vision.py`がカメラを認識しているか
3. `vision.py`の送信先が`http://127.0.0.1:8765/detections`か
4. YOLOが`litter`を検出しているか
5. `./simulation/sim.sh logs`に受信エラーがないか

トピックが一覧にあっても、Publisherがまだ送信していないと`ros2 topic echo`で型を特定できない場合があります。`vision.py`を起動して検出を発生させてから再確認してください。

### カメラを開けない、最初のクリックが反応しない

- `vision.py`など、カメラを使用中の別プロセスを終了する
- 校正画面を一度クリックしてフォーカスを移す
- `--camera 1`など別のカメラ番号を試す
- macOSの「プライバシーとセキュリティ」でターミナルのカメラ権限を確認する

### Markerが表示されない

- GazeboではなくRVizウィンドウを開く
- RVizのFixed Frameを`odom`にする
- `/litter/target_marker`をMarker表示へ設定する
- `ros2 topic info /litter/target_marker`でPublisher数を確認する
- まず`./simulation/sim.sh test-marker`で既知点を確認する

### 変換後の座標がおかしい

- クリック順が「左上、右上、右下、左下」だったか確認する
- カメラが校正後に動いていないか確認する
- 校正時とYOLO実行時の画像解像度をそろえる
- `R`で校正をやり直し、四隅と中央の結果を保存前に確認する
- 保存後にシミュレーターを再起動する

## 実機へ移行するとき

シミュレーターでは`/odom`を数値変換し、`map`座標系の`/robot/pose`（`geometry_msgs/msg/PoseStamped`）として制御へ渡します。ごみ位置も同じ変換を通った`/litter/target`（`PointStamped`、frame=`map`）です。

実機ではルンバ上面のArUcoマーカーを固定Webカメラで検出し、同じ`/robot/pose`へ差し替える想定です。画像座標から床面座標への変換と目標計算は共通で利用できます。

### Roomba 680 USB-UARTドライバ

MacとRoomba 680をFLINT FT232RLで接続し、Roomba側TXD→FLINT RXD、Roomba側RXD→FLINT TXD、GNDを共通にします。電圧レベルとMini-DINのピン配置を確認してから接続してください。既定ポートは`/dev/cu.usbserial-A10QJ1R4`、通信は19200 bps、8N1です。

実行前にRoombaの`CLEAN`ボタンを押し、ランプが点灯して本体が起きていることを確認してください。ドライバはFT232RLの安定待ち後にStartを3回送ってからSafeモードを要求します。

```bash
source .venv/bin/activate
python -m pip install -r requirements.txt
python roomba_driver.py
python roomba_driver.py --port /dev/cu.usbserial-A10QJ1R4
```

既定のCLIはSafeモードを確認して左右エンコーダを読むだけで、車輪を駆動しません。障害物のない場所で本体を床に置き、直ちに停止できる状態を確保した場合だけ、明示的に短時間の走行試験を有効にします。この試験は左右30 mm/sで0.5秒間だけ前進し、終了時にも停止指令を送ります。

実機オドメトリを読み取りだけで確認する場合は、初回値を基準として0.5秒間隔で10回測定します。左右の生値、差分tick、推定姿勢が表示され、受信エラー時は姿勢を更新せず中断します。

```bash
python roomba_driver.py --odom-test
```

```bash
python roomba_driver.py --drive-test
```

`RoombaDriver`の速度上限は既定で±50 mm/sです。OI Safeモード（センサID 35）が確認できなければ走行を拒否し、通信エラー、監視タイムアウト、`close()`、context manager終了時には可能な範囲で停止指令を送ります。Safeモードの車輪落下・崖検出等の保護は無効化しません。

ただし、Drive Directの速度指令はホストプロセスが停止すると更新も停止指令も送れません。ドライバの監視タイムアウトはプロセスとUARTが動作している間だけ有効であり、プロセス強制終了、USB抜去、OS停止時の確実な停止は保証しません。実運用では物理的な非常停止、電源遮断手段、監督プロセスを別途用意してください。

単体テストは実機へ接続せず、注入した偽シリアルで実行できます。

```bash
python -m unittest -v tests.test_odometry tests.test_roomba_driver
```

ドライバ内のオドメトリは、最初のエンコーダ読み取りを基準値とし、2回目以降に姿勢を更新します。単位は位置がm、角度がradです。

```python
with RoombaDriver() as driver:
    driver.read_odometry()  # 左右エンコーダの基準値を設定
    pose = driver.read_odometry()
    print(pose.x, pose.y, pose.theta)
```

### Roomba通信前のdry-run確認

実機へは送信せず、`/cmd_vel_unstamped`を左右輪速度へ変換して確認できます。シミュレーターのコンテナシェルで起動します。

統合経路の既定確認では、まず出力アダプターをdry-runモードで起動します。このモードは計算済みTwistを`/roomba/cmd_vel_dry_run`へ複製するだけで、実機ブリッジ入力`/roomba/cmd_vel_unstamped`へはpublishしません。

```bash
./simulation/sim.sh cmd-adapter dry_run /cmd_vel_unstamped
# 別ターミナル
./simulation/sim.sh shell
ros2 topic echo /roomba/cmd_vel_dry_run
```

アダプターの`mode`は`dry_run`、`gazebo`、`roomba`の3値です。独立したNav出力`/nav/cmd_vel_unstamped`を使う場合は、Gazebo向けを`cmd-adapter gazebo /nav/cmd_vel_unstamped`、実機ブリッジ直前を`cmd-adapter roomba /nav/cmd_vel_unstamped`で切り替えます。`roomba`はトピック接続だけであり、ブリッジの`drive_enabled=false`やPico側ロックを解除しません。

```bash
/usr/local/bin/roomba-dry-run-adapter
```

別のコンテナシェルで、`[左輪 mm/s, 右輪 mm/s]`の順に確認します。

```bash
ros2 topic echo /roomba/wheel_speeds_dry_run
```

- 前進では左右輪が同じ値になります。
- 正の`angular.z`（左旋回）では右輪が左輪より速くなります。
- 100 mm/sを超える場合は、左右輪を同じ倍率で縮小します。
- 指令が0.5秒途絶えると`[0.0, 0.0]`を配信し、古い値を再利用しません。
- `Ctrl+C`で終了するときも`[0.0, 0.0]`を配信します。

## 主なファイル

- `roomba_driver.py`: Roomba 680のSafeモードUARTドライバと実機試験CLI
- `odometry.py`: 16bitエンコーダの折り返しに対応した差動二輪オドメトリ
- `vision.py`: Webカメラ画像のYOLO推論とJSON送信
- `simulation/sim_camera_mjpeg.py`: 仮想固定カメラ画像のMJPEG配信
- `simulation/models/demo_litter/model.sdf`: 仮想カメラ検証用のゴミモデル
- `evaluate_sim_litter.py`: 既存YOLOによる仮想ゴミ評価と証跡保存
- `collect_sim_camera_points.py`: Gazebo既知点の画素座標収集
- `calibrate_sim_camera.py`: 仮想カメラ専用ホモグラフィの計算・検証
- `calibrate_floor.py`: 4点クリックによる床面ホモグラフィ校正
- `simulation/vision_turn_controller.py`: 検出座標の変換、Marker、到達判定、速度指令
- `simulation/map_frame_adapter.py`: `odom`から`map`への姿勢・目標変換と静的TF
- `simulation/roomba_dry_run_adapter.py`: 左右輪速度への変換と通信途絶停止の検証
- `simulation/cmd_vel_output_adapter.py`: Nav出力のGazebo／実機ブリッジ／dry-run切替と途絶停止
- `simulation/vision_controller.yaml`: 校正値と安全設定
- `simulation/sim.sh`: シミュレーターの起動・停止・確認コマンド
- `prepare_taco.py`: TACOデータセットの取得とYOLO形式への変換
- `simulation/README.md`: シミュレーター構成の詳細

ROS 2 Jazzy、Gazebo Harmonic、Create 3の出典と使用コミットは[simulation/README.md](simulation/README.md)に記載しています。

### Pico W TCPブリッジ

Pico W経由でRoomba 680へ接続する場合だけ、シミュレーター起動後に別ターミナルからブリッジを明示起動します。通常起動ではブリッジは起動せず、実機へ速度指令を送りません。

Wi-Fiブリッジを配備するときは、[`pico/pico_relay.py`](pico/pico_relay.py)をPico W上へ`main.py`という名前で保存します。リポジトリ内の[`pico/main.py`](pico/main.py)は既存UARTプローブとして保存してあり、自動的には置換しません。SSIDとパスワードはPico Wへ保存するローカルコピーだけへ設定し、Gitへ追加しないでください。

ホスト側は、Pico WがRoombaのOI初期化を完了して`READY`を返すまで指令を送信しません。接続直後と再接続直後にはSTOPを送信し、その接続上で新しいTwistを受信するまで停止を維持します。

```bash
# roomba_bridge_tcp.pyをDockerイメージへ反映
./simulation/sim.sh up

# 別ターミナルで、Pico W側に表示されたIPを指定
# 既定はdrive_enabled=falseなので、READY確認とSTOP送信だけを行う
./simulation/sim.sh roomba-tcp PICO_WのIPアドレス
# ポートを変更する場合
./simulation/sim.sh roomba-tcp PICO_WのIPアドレス 9000
```

正常時はPico Wに`relay ready`、ROS 2側に`relay READY; STOP sent`を含む接続ログが表示されます。既定の実機専用入力は`/roomba/cmd_vel_unstamped`（`geometry_msgs/msg/Twist`）で、Create3用`/cmd_vel_unstamped`とは分離されています。`drive_enabled=false`では非ゼロ指令を受信してもSTOP以外を送信しません。終了時は`Ctrl+C`を押すと停止指令を送ってTCP接続を閉じます。

Pico Wは500msのUART停止監視を維持します。TCP上の任意データでは監視時間を延長せず、opcode 145、固定長5バイト、左右速度がOI範囲内のDrive Directフレームを完全受信した場合だけ監視時刻を更新します。

#### STOP限定の実機疎通試験

`sim.sh roomba-tcp`は、ノード側の既定値に依存せず、起動引数として`drive_enabled:=false`と`command_topic:=/roomba/cmd_vel_unstamped`を固定します。IPアドレスとポートだけを引数で変更できます。

実機接続前に次を確認します。

- Roombaを床から浮かせるか、車輪が接地しない安定した台へ置く。
- 直ちにCLEANボタンを押せる位置で作業する。
- GP0→Roomba RX、GP1←Roomba TX、GND共通、適切なレベル変換を再確認する。
- Pico Wのシェルで`Wi-Fi OK`と`listening on <IP>:<PORT>`を確認する。
- ROS 2側で実機用の駆動許可ノードが他に起動していないことを確認する。
- `git status`でSSIDまたはパスワードを含む配備用ファイルが追加されていないことを確認する。

承認後に実行するSTOP限定コマンドは次のとおりです。このコマンド自体が実機へ接続するため、承認前には実行しません。

```bash
# 既定ポート9000
./simulation/sim.sh roomba-tcp 192.168.0.32

# 任意ポート
./simulation/sim.sh roomba-tcp 192.168.0.32 9000
```

期待ログは次のとおりです。

```text
# Pico W
connected: (...)
relay ready

# ROS 2
Connected to Pico W 192.168.0.32:9000; relay READY; STOP sent
STOP-only mode: drive_enabled=false; non-zero commands are disabled
```

別ターミナルで設定を確認できます。

```bash
ros2 param get /roomba_bridge_tcp drive_enabled
ros2 param get /roomba_bridge_tcp command_topic
ros2 topic info -v /roomba/cmd_vel_unstamped
```

期待値は`drive_enabled: false`と`/roomba/cmd_vel_unstamped`です。終了時はブリッジ側で`Ctrl+C`を押します。正常ならホストがSTOPを送って切断し、Pico Wも切断処理でSTOPを送ります。ホストが応答しない場合はPico W側で`Ctrl+C`を押し、それでも停止を確認できない場合はRoombaのCLEANボタンで停止してください。Wi-Fi切断時はPico Wの500ms watchdogによる停止を待ちますが、Pico W自体の停止・電源断時はソフトウェア停止を保証できません。

#### OIモードとSTOP到達の診断

診断対応版の`pico_relay.py`を配備した後、承認を得てから次を実行します。通常の`roomba-tcp`と同様に`drive_enabled=false`および実機専用トピックが固定され、非ゼロDrive DirectはホストとPico Wの両方で拒否されます。

```bash
./simulation/sim.sh roomba-tcp-diagnostic 192.168.0.32 9000
```

診断プロトコルは任意UART転送ではありません。既存のDrive Direct固定長5バイトに加え、次の2種類だけを許可します。

| TCP要求 | 用途 | UART操作 |
|---|---|---|
| `D7 01` | STOP到達確認 | STOPを書き、センサ41/42を固定Query Listで読む |
| `D7 02` | OIモード確認 | 先にSTOPを書き、センサ35だけを読む |

STOP ACKの状態ビットは、`0x01`がPicoでの受信・検証、`0x02`がUARTへの5バイト書込み、`0x04`がRoombaの要求右・左車輪速度センサが両方0だったことを示します。`0x07`かつ左右0の場合だけ成功です。期待ログは次のとおりです。

```text
Connected to Pico W 192.168.0.32:9000; relay READY; STOP sent
STOP confirmed: host_sent=true, pico_validated=true, uart_written=true, roomba_requested_right=0, roomba_requested_left=0, ack=0x07
Roomba OI mode: 2 (Safe)
STOP-only mode: drive_enabled=false; non-zero commands are disabled
```

OIモード値は`0=Off`、`1=Passive`、`2=Safe`、`3=Full`です。2以外はエラーログとし、走行は引き続き無効です。診断要求、ACK、センサー応答はPico Wの走行watchdogを延長しません。診断処理は最初にSTOPを送り、UART応答待ちは有限時間です。

センサ41/42の0は、Roomba内部の「要求車輪速度」が0になったことを示す間接確認です。実際の車輪回転や床上速度を直接測定するものではありません。慣性による回転まで確認する場合は、エンコーダ43/44を時間差で読み、tick変化が停止したことを別途検証する必要があります。

## 既知座標による自律移動試験（第4段階A）

YOLOを使わず、Gazeboの既知座標を入力して共通Navigationを検証できます。通常の`sim.sh up`では起動せず、次の明示的な手順だけで制御を有効化します。

```bash
./simulation/sim.sh nav-test-up
./simulation/sim.sh known-target-test
```

テスト入力は`odom`座標系の`/litter/target_odom`へ送ります。`map_frame_adapter`が`/litter/target`へ変換するため、通常の目標経路と競合せず、`vision_turn_controller`の制御計算をそのまま通ります。試験前に仮想カメラ用ホモグラフィの検証点誤差、初期安全フラグ、最高速度0.08 m/s以下を検査します。

シミュレーション専用設定では到達判定に1 mmの`arrival_epsilon`を使用します。比例速度が到達半径へ漸近して停止する数値境界だけを吸収する設定で、通常設定は`arrival_epsilon=0.0`のため従来動作です。到達後は同一目標に対する到達状態を保持し、ゼロ速度を明示的に配信し続けます。

結果は`artifacts/known_target_test/result.json`へ保存されます。初期・目標・最終姿勢、軌跡、最終位置誤差、旋回・前進・到達・停止、入力タイムアウト停止、停止理由を含みます。例外終了時の停止も別に確認できます（このコマンドは意図どおり非0で終了します）。

```bash
./simulation/sim.sh nav-test-up
./simulation/sim.sh known-target-abort-test
```

どちらも終了処理で`control_enabled=false`、`calibration_ready=false`へ戻し、ゼロ速度を送信します。この試験はYOLO認識の成功を示すものではなく、Roomba 680へは接続しません。通常環境へ戻すには`./simulation/sim.sh up`を実行します。

前後振動を読み取り専用で記録する場合は、専用試験を開始した別ターミナルで次を実行します。

```bash
./simulation/sim.sh oscillation-diagnostics 190
```

`/cmd_vel_unstamped`、`/cmd_vel`、`/diffdrive_controller/cmd_vel`、`/odom`、`/hazard_detection`を同一の単調時計基準で記録し、`artifacts/oscillation_diagnostics/`へJSONL、ROSグラフ、反転一覧を保存します。この診断ノードはPublisherを持ちません。

## 複数目標への自律追従試験（第4段階B）

複数の既知目標を順番に追従する試験も、自由空間配置でのみ明示的に実行します。

```bash
./simulation/sim.sh nav-test-up
./simulation/sim.sh multi-target-test
```

試験中は実カメラ検出との競合を避けるため、試験専用コンテナ内の`webcam-bridge`を停止します。A・Bの到達、同一目標再配信時のラッチ保持、新目標でのラッチ解除、各到達後2.2秒のゼロ速度、入力タイムアウト、安全フラグの後処理を検証します。結果は`artifacts/multi_target_test/result.json`と`REPORT.md`へ保存されます。
# Gazebo YOLOパイロット学習（Google Colab）

学習はMacでは実行せず、MacはGazebo画像・YOLOラベルの生成とZIP作成だけを担当します。

```bash
./simulation/sim.sh vision-integration-up
.venv/bin/python build_gazebo_litter_dataset.py
./simulation/sim.sh stop
.venv/bin/python package_gazebo_litter_colab.py
```

生成物は `artifacts/gazebo_litter_pilot_colab.zip` です。このZIPと
`notebooks/gazebo_litter_pilot_colab.ipynb` をGoogle Driveの
`MyDrive/robot-1-colab/`へアップロードしてください。ZIPにはデータセット、評価スクリプト、
既存重みのコピー `weights/original_best.pt` が含まれます。元の
`runs/detect/taco/weights/best.pt`は変更されません。

ColabではランタイムをGPUへ変更し、ノートブックを上から順に実行します。既定値は
`imgsz=640`、`batch=8`、`epochs=50`、`patience=10`、`seed=42`、`device=0`です。
CUDAメモリ不足の場合だけbatchを8→4→2の順に下げます。各epochの`last.pt`はDriveへ
保存され、ノートブック内の再開セルから`resume=True`で再開できます。

完了後、Drive上の`gazebo_litter_pilot_result.zip`をMacへ戻してください。ZIPには
`weights/best.pt`、`weights/last.pt`、学習曲線、固定test holdoutでの旧新モデル比較が
含まれます。Navigationを有効にする前に比較結果を確認してください。
