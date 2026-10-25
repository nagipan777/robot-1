# ArUco位置合わせ再開手順書

更新日: 2026-10-20。実機を走行させず、候補姿勢だけを検証する。

## 1. 現行構成

- カメラ: Logicool C270、固定、要求解像度1280×960。
- 辞書: `DICT_4X4_50`、Marker ID 0。
- 入力: `vision.py`の`aruco_detections`（順序付き4隅pixel）。
- 床面変換: `simulation/aruco_pose.yaml`の保存済みimage→map homography。
- 候補: `/robot/pose_candidate`、合格fix: `/localization/aruco_pose`。
- 最終出力: `/robot/pose` (`PoseStamped`, `frame_id=map`)。既定では無効。
- `pose_calibration_ready=false`、`pose_output_enabled=false`を維持する。

## 2. 最小カメラ確認（YOLO・Gazebo・ROS2なし）

Macで他のカメラアプリを終了し、AC電源を接続する。次はYOLOモデルを読み込まず、HTTP送信もしない。

```bash
.venv/bin/python vision.py \
  --camera 0 --width 1280 --height 960 \
  --preview --simulator "" \
  --aruco-marker-id 0 --aruco-dictionary DICT_4X4_50
```

枠が安定して描画されることだけを確認し、`q`またはEscで終了する。カメラ番号が異なる場合だけ
`--camera 1`を試す。同時にGazebo、YOLO学習、MJPEGを起動しない。

ROS座標候補を確認する段階では既存HTTP→`/webcam/detections`と`aruco-pose`を使用するが、
`pose_output_enabled=false`のまま `/robot/pose_candidate` と `/robot/pose_status`だけを記録する。

## 3. マーカー取付け記録

以下は測定完了まで`UNKNOWN`とする。

| 項目 | 現在値 | 扱い |
|---|---:|---|
| dictionary / ID | DICT_4X4_50 / 0 | 確定 |
| 黒枠一辺 | 0.0 m | 要実測。印刷拡大縮小禁止 |
| top-plane高さ | YAML 0.08 m | 要再測定 |
| marker中心→機体中心 forward/left | 0/0 m | 要測定 |
| marker x軸→Roomba前方yaw offset | YAML 3.079158 rad | 要独立検証 |
| camera高さ/床面投影点 | 0.82 m / (1.7, 0.1) m | 要設置記録と再測定 |

マーカーは上面へ平坦・剛固定し、反射を避ける。Roomba前方、marker x軸、四隅順序を写真へ残す。

## 4. 座標系と変換

```text
image pixel corners
  -- saved homography --> floor intersection in map
  -- camera/marker height correction --> elevated marker center in map
  -- yaw + mount offset --> base_link origin in map
  --> /robot/pose_candidate (map)

encoder 43/44 --> odom->base_link prediction
ArUco map->base_link + odom->base_link --> map->odom correction
```

- `map`: 固定カメラ校正で定義した床面座標、m。
- `odom`: encoder積分開始時の局所座標。時間とともにドリフトする。
- `base_link`: Roomba機体中心。marker中心と同一とは仮定しない。
- yawはmap +xから反時計回りを正とし、`[-pi, pi]`へ正規化する。
- homographyは床面用。高さのあるマーカーへ直接適用すると位置がずれるため、カメラ床面投影点、
  camera高さ、marker高さを用いた補正が必要。
- 内部パラメータと歪み係数が未取得なので、完全な3D PnP姿勢または保証済み絶対精度とは扱わない。

## 5. 静止検証

1. カメラ脚、焦点、解像度、向き、床上基準点を写真と寸法で記録する。
2. Roombaを既知の独立検証点へ手で置き、走行ロックは全てfalseにする。
3. ID、4隅、pixel面積、timestampを最低10秒記録する。
4. x/y/yawの平均、最大偏差、標準偏差、欠落率を計算する。
5. map +x/+yとRoomba前方を手移動・手回転で確認する。
6. マーカー遮蔽で`LOST`、古いtimestampで`REJECTED`となることを確認する。
7. map範囲外、非有限値、yaw跳躍、複数`/robot/pose` Publisherを拒否する。
8. 校正に使っていない位置と4方位で誤差を記録する。許容値は実測後に承認し、後付けしない。

確認コマンド:

```bash
./simulation/sim.sh aruco-pose
./simulation/sim.sh shell
ros2 topic echo /robot/pose_candidate
ros2 topic echo /robot/pose_status
ros2 topic info -v /robot/pose
```

`/robot/pose`のPublisherが既存Gazebo/手動/fusionと競合する場合は出力を有効化しない。

### 提出前の確定手順

この確認ではRoombaを手で既知位置へ置くだけで、TCP 9000、UART、走行ロックを使用しない。

1. ROS側を実機dry-run構成で起動する。Pico IPは指定しない。

   ```bash
   cd /Users/imaichika/robot
   ./simulation/sim.sh real-camera-dry-run-up
   ./simulation/sim.sh aruco-pose-start
   ```

2. Mac側でYOLOを読み込まず、ArUco ID 0だけを1280×960で検出・送信する。

   ```bash
   cd /Users/imaichika/robot
   .venv/bin/python vision.py \
     --camera 0 --width 1280 --height 960 --preview \
     --aruco-marker-id 0 --aruco-dictionary DICT_4X4_50 \
     --http-timeout 2.0
   ```

3. コンテナ内の別シェルで候補と状態を最低10秒確認する。

   ```bash
   ./simulation/sim.sh shell
   ros2 topic echo /robot/pose_candidate
   ros2 topic echo /robot/pose_status
   ros2 topic info -v /robot/pose
   ```

PASS条件は、ID 0が継続検出され、`frame_id=map`、timestampが進み、静止中のx/y/yawに
大きな跳躍がなく、遮蔽時にLOSTまたはREJECTEDとなり、`/robot/pose`へ競合Publisherを
追加しないことである。marker実寸、高さ、offset、独立検証点誤差が未確定な間は、候補値が
安定していても絶対姿勢校正PASSとはしない。

4. 終了する。

   ```bash
   ./simulation/sim.sh aruco-pose-stop
   ./simulation/sim.sh down
   ```

## 6. オドメトリ併用

- Query List `149,2,43,44`、左右各unsigned 16-bit big-endian。
- wraparound差分は既存`odometry.py`/`localization_core.py`を再利用する。
- 車輪径、508.8 counts/rev、車輪間距離0.235 mはいずれも暫定値。
- odometryは予測、ArUcoはmap補正として扱い、異なるframeの値を直接平均しない。
- ArUco鮮度切れは短時間`ODOM_ONLY`、odom切れ・上限時間超過は`LOST`として出力停止する。
- encoderの実UART取得は未実装・未検証なので、現在はmock previewだけを使う。

```bash
./simulation/sim.sh localization-preview-test
```

## 7. 校正再利用条件

保存対象は、カメラ機種/serial、解像度、設置写真、三脚位置・高さ・yaw/pitch、床基準点、homography、
marker印刷寸法、高さ、機体offset、yaw offset、検証日時、独立点誤差、設定ファイルhash。

次のどれかが起きたら再校正する。

- カメラ、三脚、焦点、解像度、クロップ、向きが変わった。
- 床基準点またはmap原点が変わった。
- マーカーを再印刷・再固定した、marker高さ/offsetが変わった。
- 独立検証点が許容値を超えた、または画像歪み・振動が増えた。

再利用時も毎回、最低1つの未使用検証点と既知yawでチェックし、合格するまで
`pose_calibration_ready=false`、`pose_output_enabled=false`を維持する。
