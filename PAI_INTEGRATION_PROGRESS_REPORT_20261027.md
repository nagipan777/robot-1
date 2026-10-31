# PAI最終課題 実機統合・通信修復・オドメトリ検証報告書

作成日：2026年10月27日  
対象：Roomba 680、Pico W、ROS 2、YOLO、ArUco  
作業ブランチ：`fix-arm-ack-session-recovery`

## 1. 概要

本作業では、AI認識からRoomba実機動作へ至る統合経路について、安全条件を維持したまま通信障害の原因究明と修復を行った。さらに、Roombaの左右エンコーダを用いる観測専用オドメトリを、実機を動かさず検証できるROS 2 dry-run経路として実装した。

主要成果は次のとおりである。

1. 10月26日の短縮E車輪浮上試験で発生したARM ACK `0/7` timeoutを再現し、Pico側送信処理の欠陥を原因として特定した。
2. ARM ACK送信、ARM状態遷移、TCPセッション識別、切断後再接続を修復した。
3. 修正版を実Picoへ配備し、走行ロック有効・Roomba未接続の非走行試験で10/10成功を確認した。
4. オドメトリ計算、CSV replay、ROS 2 `/roomba/odom`、CSV記録、RViz設定、ArUco A/B比較を実装した。
5. ネットワークを遮断したコンテナでROS 2 E2EをPASSし、利用者による同一コマンドの再実行でもPASSを確認した。
6. リポジトリ全体の回帰テスト338件がすべてPASSした。

現時点で、通信修復とオドメトリdry-runは完了している。一方、実機エンコーダ取得、実測ArUco比較、および短縮Eの再走行は未実施であり、本報告のPASSから実機走行許可を自動的に導かない。

## 2. 維持した安全条件

以下は変更・緩和していない。

- ARMゲート
- STOPラッチ
- Picoの500 ms watchdog
- UART部分書込み・異常時のSTOPと走行禁止
- 実機速度上限20 mm/s
- 短縮Eのホスト側0.4秒ゲートと最大1秒制限
- 50 cm近傍ガード
- ArUco姿勢鮮度判定
- STOP後の新規ARM要求
- 人間による実機試験の最終承認

Picoの既定値は `ALLOW_NONZERO_DRIVE=False`、`BRC_WAKE_ENABLED=False` である。BRCは使用せず、実機起床はCLEANボタンによる手動操作を前提とした。

## 3. 短縮E試験で発生した障害

### 3.1 実測結果

10月26日の車輪浮上試験では、初期診断として次を確認した。

- TCP READY：成功
- STOP ACK：`0x07`
- Roomba要求速度：右0、左0
- OI mode：2（Safe）
- Pico側ARM generation受理：確認

しかし、Mac側はARM ACKを1 byteも受信できず、`0/7 bytes`、2.008秒でtimeoutとなった。その後、READY timeoutとTCP切断が連続した。Drive ACKはなく、車輪の物理回転もなかった。終了時にはSTOP、左右0、Safe mode 2、およびPico・ROS双方の走行ロック無効状態を再確認した。

この試行の判定はFAILである。ただし、非ゼロDriveが成立せず、車輪が回転せず、安全側へ復帰したことは確認できた。

### 3.2 原因

確認された直接原因は、MicroPythonのnon-blocking `socket.send()` が一時的に `None` を返す場合に、旧 `send_all()` が残り全byteの送信完了として扱っていたことである。

このため、Pico内部ではARMを受理したように見えても、実際には7 byteのACKがTCPへ書き込まれない状態を作り得た。ホスト側timeoutを延長しても直らない送信完了判定の欠陥であった。

同時期にWi-Fi RSSI `-82 dBm`、30 ping中56.7% lossも観測された。これはREADY不安定化の寄与要因だが、ACK `0/7`を発生させるコード欠陥とは区別した。

## 4. ARM/TCP修復

### 4.1 修正内容

- `send(None)` とwould-blockを既存100 ms期限内で再試行する。
- partial writeを実送信byte数で追跡する。
- 全7 byte送信完了後にのみARMを確定する。
- 状態遷移を `STOPPED/LATCHED_STOP -> ARM_PENDING -> ARMED` とする。
- ARM generationをTCP connection IDへ結び付ける。
- ACK失敗、期限切れ、STOP、例外、切断時はSTOPラッチへ遷移する。
- ACK未達のgenerationを再利用しない。
- 再接続後に旧セッションのARMを利用できないようにする。
- ホストはACK全7 byteの検証前にARM成功と扱わない。
- READY後500 ms以内に最初のframeがない接続を解放し、再接続可能にする。
- ACK失敗時の自動ARM再送・Drive再送は追加しない。

ARM期限、500 ms watchdogおよびホストACK timeoutは延長していない。

### 4.2 検証結果

| 検証 | 結果 |
|---|---:|
| ARM/TCP集中回帰 | 105/105 PASS |
| 最終host側非走行回帰 | 131/131 PASS |
| Mac正常mock TCP | 300/300 PASS |
| Colima正常mock TCP | 300/300 PASS |
| Mac長時間stress | 5,000/5,000 PASS |
| Colima長時間stress | 5,000/5,000 PASS |
| fault matrix | timeout、partial close、completeを全件期待どおり分類 |
| 実Pico・ロック拒否ACK/再接続 | 10/10 PASS |

実Picoの最終10回では、ARM ACK平均96.8 ms、最大274.5 ms、初回READY平均1.232秒、再接続READY平均1.274秒であった。timeout、partial ACK、Drive frame、非ゼロ車輪指令はいずれも0件である。

## 5. 観測専用オドメトリ

### 5.1 目的と境界

Roomba OIセンサーpacket 43・44の左右エンコーダ値を将来入力できるよう、差動二輪オドメトリを独立した観測機能として実装した。今回の検証入力はCSVによる模擬値であり、実機UARTは使用していない。

```text
CSV replay
  -> /roomba/encoder_counts (std_msgs/msg/String)
  -> wheel_odometry_node
  -> /roomba/odom (nav_msgs/msg/Odometry)
  -> CSV保存 / ArUco A/B比較
```

- 初期状態：`odometry_enabled=false`
- frame：`roomba_odom`
- child frame：`roomba_base_link`
- TF：発行しない
- `/robot/pose`：変更せず、比較入力としてのみ利用
- Navigation：接続しない
- Pico、UART、ARM、STOP、Drive：変更しない

### 5.2 実装機能

- 差動二輪モデルによるx、y、yaw積分
- linear/angular velocity計算
- 16 bit encoder wraparound
- 左右encoder delta記録
- 不正値・大きなjump・非単調timestampの拒否
- timeout・切断中の積分停止
- 再接続および長時間gap後の再基準化
- CSV即時または時間倍率付きreplay
- `/roomba/odom`のpose、twist、timestamp、covariance発行
- ROS出力と比較情報のCSV保存
- RViz用設定
- ArUco初期SE(2)姿勢除去
- x/y線形補間と最短角yaw補間
- 位置・方位のRMSE、最大誤差、終端誤差、時刻差、未対応数の算出

固定シナリオとして前進、後退、停止、左右旋回、wraparound、異常jump、通信断・再接続、理論26.8度旋回を用意した。26.8度旋回は模擬値であり、実測成功とは扱わない。

## 6. dry-run E2E結果

再現コマンドは次の1つである。

```bash
./simulation/sim.sh odometry-dryrun-e2e
```

専用ROS 2コンテナは `--network none` で起動するため、PicoやRoombaへ接続できない。CSV publisher、ROS 2 subscriber、`/roomba/odom` publisher、CSV保存、模擬ArUco A/B比較までを一括実行する。

### 6.1 ROS 2結果

| 項目 | 結果 |
|---|---:|
| 入力sample | 6 |
| `/roomba/odom` message | 5 |
| timeout入力 | 1件、積分せずPASS |
| 最終x | 0.0142259145 m |
| 最終y | 0.0000538245 m |
| 最終yaw | 0.0075670638 rad |
| message型 | `nav_msgs/msg/Odometry` |
| TF発行 | false |
| Navigation接続 | false |
| 実UART | false |

### 6.2 模擬ArUco A/B結果

| 指標 | 結果 |
|---|---:|
| 対応sample | 5 |
| 対応失敗 | 0 |
| 位置RMSE | `1.3449e-16 m` |
| 方位RMSE | `7.6209e-18 rad` |
| 最大位置誤差 | `1.9892e-16 m` |
| 最大方位誤差 | `1.0408e-17 rad` |
| 平均・最大時刻差 | 0.0 s |

誤差は同一理論軌跡から生成した模擬ArUcoデータに対する浮動小数点丸め範囲である。実機精度を示す数値ではない。

### 6.3 再現確認

CodexによるE2E成功後、ユーザー自身が同じコマンドを実行し、次の完了表示を確認した。

```text
PASS: odometry dry-run E2E artifacts saved to /Users/imaichika/robot/artifacts/odometry_dryrun/e2e
```

したがって、開発環境内での再現性をPASSと判定する。

## 7. 総合テスト結果

- オドメトリ集中テスト：29/29 PASS
- リポジトリ全回帰：338/338 PASS
- ARM/TCP mock・stress：PASS
- 実Picoロック状態のACK・再接続：10/10 PASS
- ROS 2 dry-run E2E：PASS
- 利用者によるdry-run再実行：PASS

全回帰の初回sandbox実行では、314件成功、24件がlocalhost socket権限制限でERRORとなった。同一テストをloopback許可環境で再実行し338/338 PASSを確認したため、製品コードのFAILとしては扱わない。

## 8. 成果物

主要文書：

- `ARM_TCP_RECOVERY_20261026.md`
- `NIGHTLY_ARM_TCP_REPORT_20261027.md`
- `P3_P7_STATUS_20261027.md`
- `ROOMBA_ODOMETRY_PROTOTYPE.md`
- `ODOMETRY_DRYRUN_REPORT_20261027.md`
- 本報告書

主要ログ・結果：

- `artifacts/odometry_dryrun/e2e/ros_e2e_summary.json`
- `artifacts/odometry_dryrun/e2e/ros_odometry.csv`
- `artifacts/odometry_dryrun/e2e/offline/summary.json`
- `artifacts/odometry_dryrun/e2e/offline/comparison.csv`
- `artifacts/odometry_dryrun/e2e/offline/trajectory.svg`
- `artifacts/odometry_dryrun/e2e/unittest-338.log`

GitHub：

- リポジトリ：`nagipan777/robot-1`
- ブランチ：`fix-arm-ack-session-recovery`
- PR：`#3`
- ARM/TCP修復基点：`5270832`
- 実Pico検証：`53ec17e`
- オドメトリdry-run：`36488e0`
- mainへのマージ：未実施

## 9. 現在の判定

### 開発・非走行診断

**GO**。ARM/TCP修復、mock試験、実Picoロック試験、オドメトリdry-runの追加開発を継続できる。

### オドメトリdry-run

**PASS**。実機を使わず、一つのコマンドでROS 2出力、CSV、可視化用成果物、ArUco A/B比較を再現できる。

### 短縮E車輪浮上試験の再走行

**現時点では自動GOとしない**。過去のFAIL原因であったARM ACK送信経路は修復済みだが、試行当日に以下を再確認する必要がある。

- Pico配備版とリポジトリ版の一致
- `ALLOW_NONZERO_DRIVE=False`から始まること
- BRC未接続、CLEAN手動起床
- 両輪完全浮上、独立停止手段準備
- STOP ACK `0x07`、左右要求速度0、OI Safe mode 2
- 時計差1秒未満
- ArUco ID 0の新鮮な姿勢
- YOLO 3連続検出と50 cmガード
- 実機速度上限20 mm/s
- 独立0.4秒ゲートと500 ms watchdog
- STOP後の再ARM禁止
- Gazebo/Create3停止、物理出力adapterが1つだけ
- 人間による当該1試行の明示承認

## 10. 未完了事項と次段階

### 今回の実機試験を禁止する具体的ブロッカー

試行直前のSTOP/Safe/車輪0、姿勢、検出、安全支持、出力経路が未確認の状態では実機走行を禁止する。解除方法は、既存runbookに従って各項目を非走行で一つずつ実測し、全項目PASS後に人間が1試行を承認することである。

### 限定試験中に監視できる事項

- Wi-Fi遅延と単発再接続
- ACK応答時間
- encoder/ArUcoのsample欠損
- CSV時刻差
- 模擬値と実測値の差

これらはログを取得しながら管理できるが、ACK timeout、通信断、姿勢喪失、時間超過、異常回転が本番試行で発生した場合は即中止し、その試行中に再ARM・再Driveしない。

### 提出後へ延期可能な改善

- オドメトリとNavigationの融合
- 実機用covarianceの最適化
- 長時間走行のdrift補正
- BRC自動起床
- generationの不揮発化

## 11. 結論

短縮E試験で顕在化したARM ACK消失は、Picoの送信完了判定とARM確定順序の問題として特定・修復された。実Picoのロック状態で10/10のACK・再接続成功を確認し、既存安全ゲートを維持した。

加えて、観測専用オドメトリは、実機と切り離されたROS 2 dry-runとして完成した。計算、異常処理、CSV replay、`/roomba/odom`、RViz設定、模擬ArUco A/B比較、全回帰、および利用者による再現までPASSしている。

したがって、通信修復および非走行開発は成果として確定できる。次の実機工程では、dry-runの成功を走行許可と混同せず、短縮Eの当日安全条件を再確認した上で、別途承認された単発試行として実施する。
