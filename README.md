# robot-1 チュートリアル

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

```text
Webカメラ
  └─ vision.py（YOLO）
       └─ HTTP → シミュレーター
                    └─ /webcam/detections（std_msgs/msg/String、JSON）
                         └─ vision_turn_controller
                              ├─ /odom（ロボット姿勢）
                              ├─ /litter/target（変換後の目標点）
                              ├─ /litter/target_marker（RViz Marker）
                              ├─ /litter/reached（到達判定）
                              └─ /cmd_vel_unstamped（現在は出力無効）
```

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
python vision.py
```

映像ウィンドウも表示する場合は次を使います。

```bash
python vision.py --preview
```

`vision.py`は既定でシミュレーターの`http://127.0.0.1:8765/detections`へ検出JSONを送ります。送信せずカメラとYOLOだけを確認する場合は、空の送信先を指定します。

```bash
python vision.py --simulator '' --preview
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

## 9. ロボットを手動で動かす

自動走行を有効にせず、キーボード操作で座標と到達判定を確認できます。

```bash
./simulation/sim.sh teleop
```

操作方法はターミナルに表示されます。ごみ目標へ15 cm以内まで近づくと、`/litter/reached`が`true`になります。

## 10. 終了する

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

シミュレーターではロボット姿勢として`/odom`を使います。実機ではルンバ上面のArUcoマーカーを固定Webカメラで検出し、校正済み世界座標系の`geometry_msgs/msg/PoseStamped`として渡す想定です。

その際は`simulation/vision_controller.yaml`で`pose_source: pose_stamped`へ変更し、`pose_topic`にArUco姿勢トピックを指定します。画像座標から床面座標への変換と目標計算は共通で利用できます。

## 主なファイル

- `vision.py`: Webカメラ画像のYOLO推論とJSON送信
- `calibrate_floor.py`: 4点クリックによる床面ホモグラフィ校正
- `simulation/vision_turn_controller.py`: 検出座標の変換、Marker、到達判定、速度指令
- `simulation/vision_controller.yaml`: 校正値と安全設定
- `simulation/sim.sh`: シミュレーターの起動・停止・確認コマンド
- `prepare_taco.py`: TACOデータセットの取得とYOLO形式への変換
- `simulation/README.md`: シミュレーター構成の詳細

ROS 2 Jazzy、Gazebo Harmonic、Create 3の出典と使用コミットは[simulation/README.md](simulation/README.md)に記載しています。
