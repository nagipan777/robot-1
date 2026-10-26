# 短縮E車輪浮上試験

## 目的と範囲

実WebカメラのYOLO検出、ArUco実測姿勢、ROS 2 Navigation、TCP 9000、Pico W、
Roomba 680の順に接続し、両輪浮上状態でのみ物理回転を確認する。本試験は床面自律走行ではない。

通常の45秒E試験を物理出力へ直結しない。短縮E専用ブリッジは最初の非ゼロ指令を起点に
ホスト側の許可時間を0.4秒に制限し、期限時にSTOPを送ってARMを破棄する。0.4秒は、
確認済みのPico watchdog約0.503秒がバックアップ停止になった場合でも合計約0.903秒となるよう、
1秒上限に余裕を設けた値である。

ソフトウェアだけで車輪の物理停止時刻は保証しない。動画または目視で停止時刻を確認し、
STOP書込みや停止を確認できない試行はFAILとする。

## 実装確認

- `simulation/roomba_bridge_tcp.py`
  - `max_drive_session_duration`の既定値は`0.0`（通常経路では無効）。
  - 有効値は0より大きく1.0秒以下だけを受け付ける。
  - 最初の非ゼロ指令を単調時計で記録する。
  - コールバック受信時と20Hzタイマーの両方で期限を検査する。
  - 期限時に指令を破棄し、Drive Directゼロを送り、ホスト側ARMを破棄する。
  - 新規ARMまで後続非ゼロ指令を拒否する。
- `simulation/sim.sh roomba-short-e-bridge`
  - `WHEELS_LIFTED`指定がなければ起動を拒否する。
  - `max_speed_mm_s=20`、`command_timeout=0.5`、時間ゲート`0.4`秒を固定する。
  - この起動コマンド自体はARMも非ゼロ指令も送らない。
- Pico W
  - 500ms watchdog、世代・sequence・CRC、UART全5バイト書込み、STOPラッチを維持する。
  - Pico側コードとTCP 9000プロトコルは本対応では変更しない。

`command_timeout=0.5`だけでは、Navigationが連続Publishする限り期限が更新されるため、
1秒制限にはならない。短縮Eでは独立した0.4秒の一回限りゲートを使用する。

## 2026-10-25 ROS 2配送遅延の事前検証

実機未接続、`drive_enabled=false`、localhostモックの条件で、実際の
`vision_turn_controller`が指令を計算した直後から、ブリッジ受信および安全拒否までを
`time.monotonic_ns()`で計測した。各条件20標本、合計80標本を取得した。

| 条件 | 指令生成→ブリッジ受信 平均 | 最大 | 受信/拒否 | 実機Drive |
|---|---:|---:|---:|---:|
| 軽量 | 1.025 ms | 2.753 ms | 20/20 | 0 |
| YOLOのみ | 3.181 ms | 27.377 ms | 20/20 | 0 |
| Gazeboのみ | 1.994 ms | 7.190 ms | 20/20 | 0 |
| Gazebo＋YOLO | 6.590 ms | 23.017 ms | 20/20 | 0 |

全80件が`drive_enabled=false`で拒否され、TCP Drive、UART Drive、Drive ACKは0件だった。
以前観測した約1.65秒の遅延は再現しなかった。今回の最大値は27.377msだが、20標本/条件のため、
まれな長時間遅延が存在しないことまでは証明しない。カメラ取得、HTTP、Wi-Fi、Pico、UARTも
この測定範囲には含まれない。

生ログと集計は次に保存している。

- `artifacts/navigation_latency_20261025/REPORT.md`
- `artifacts/navigation_latency_20261025/{lightweight,yolo-only,gazebo-only,gazebo-yolo}.jsonl`
- 同ディレクトリの`*-summary.json`、`*-bridge.log`、`*-navigation.log`
- 全回帰ログ: `artifacts/navigation_latency_20261025/unittest-295.log`

## 1秒制限と遅着指令の非走行検証

短縮Eの0.4秒ゲートは最初の非ゼロ指令で一度だけ開始し、後続のNavigation指令では期限を
更新しない。ROS Executorが止まった場合はホストから新しいDriveが出ず、最後のDriveから
Pico watchdog 500msでSTOPする。ホストが遅れて復帰した場合も、ブリッジは指令をキューへ
入れる前に固定期限を検査し、期限切れとしてSTOPラッチへ遷移する。

したがって、Picoが給電されwatchdogが動作している前提での論理上限は
`0.4秒 + 0.5秒 = 0.9秒`であり、ROS処理停止によって1秒を超えるよう期限が延長される構造ではない。
ただし、Pico電源断、UART断、Roomba側のコマンド解釈異常時の物理停止はこの計算では保証しない。

次の非走行テストで確認した。

- 0.399秒までの反復指令で開始時刻が更新されない。
- 1.65秒停止後に届いた非ゼロ指令が`expired`となり、その後`latched`になる。
- STOP後に0.001秒、0.76秒、1.65秒、10秒遅れて届いた旧指令がすべて拒否される。
- ラッチ解除には独立した新規ARMが必要である。
- Pico mockで500ms watchdogがDrive Directゼロを1回だけUARTへ書き、ラッチ後の非ゼロを拒否する。

ARM期限とPico watchdog値は変更していない。

## GO / NO-GO事前判定

| 項目 | 必須条件 | 現在の証拠 | 判定 |
|---|---|---|---|
| ROS配送 | 期限切れを通さず、負荷下でも安全拒否 | 4条件×20件、全件受信・拒否、最大27.377ms | PASS |
| 時間ゲート | 固定0.4秒、反復指令で延長しない | pure unit test | PASS |
| 遅着指令 | STOP後は新規ARMまで拒否 | pure unit test | PASS |
| Pico watchdogロジック | 500msでUART STOP、ラッチ | Pico mock test | PASS |
| 両輪浮上 | 安定した支持台、車輪・ブラシ周辺に物なし | 試行ごとに人間が確認 | UNKNOWN |
| 手動起床 | BRCなし、CLEANで起床 | 試行ごとに人間が確認 | UNKNOWN |
| Pico配備版 | リポジトリ版と一致し、UART1 GP4/GP5、115200bps、watchdog 500ms、世代/CRC/ACK | 現在の実機ファイル照合なし | UNKNOWN |
| STOP診断 | 最新試行でACK 0x07、right=0、left=0、OI Safe=2 | 今回は実機未接続 | UNKNOWN |
| UART Drive ACK | 現配備版で0x03、部分書込み・timeoutなし | 今回は実機Drive禁止 | UNKNOWN |
| ArUco | ID 0、姿勢鮮度1秒以内 | 試行直前に確認 | UNKNOWN |
| YOLO | confidence 0.50以上、3連続検出、50cmガード通過 | 試行直前に確認 | UNKNOWN |
| 実機Publisher | 意図した出力アダプター1つだけ | 試行直前に確認 | UNKNOWN |
| 速度制限 | ブリッジ`max_speed_mm_s=20` | launcher/codeで固定 | PASS |
| 独立停止 | 監視者がCLEAN/電源操作可能 | 試行直前に確認 | UNKNOWN |

現在の最終判定は**NO-GO（HOLD）**。ソフトウェアのROS配送、固定期限、STOPラッチ、
watchdog mockはPASSしたが、Pico実配備版の一致、最新STOP限定診断、現配備版でのDrive ACK 0x03、
物理支持などの試行固有条件が未確認である。これらを推測でPASSにしない。

全UNKNOWNが試行直前にPASSした場合だけ、その1回の短縮E車輪浮上試験をGOとする。

## 事前準備（走行ロックを解除しない範囲）

リポジトリからコンテナを更新する。

```bash
cd /Users/imaichika/robot
docker --context colima-create3 compose -f simulation/compose.yaml build simulator
./simulation/sim.sh real-camera-aruco-dry-run-up
./simulation/sim.sh shell
```

コンテナ内でシミュレーションを停止し、必要な4プロセスだけを残す。

```bash
supervisorctl stop create3 unstamped-cmd-relay presentation-visualizer
pkill -TERM -f 'gz sim' || true
pkill -TERM -f '/ros_gz_bridge/' || true
pkill -TERM -f '/irobot_create_nodes/' || true
pkill -TERM -f '/irobot_create_gz_toolbox/' || true
sleep 3
pgrep -af 'gz sim|ros_gz|irobot_create|motion_control|controller_manager' || true
supervisorctl status webcam-bridge vision-turn-controller aruco-pose cmd-vel-output-adapter
```

`pgrep`は空、必要な4プロセスは`RUNNING`でなければならない。

Mac側で実カメラ処理を起動する。

```bash
cd /Users/imaichika/robot
.venv/bin/python vision.py \
  --camera 0 --width 1280 --height 960 \
  --weights runs/detect/real_finetune/weights/best.pt \
  --confidence 0.50 \
  --aruco-marker-id 0 \
  --aruco-dictionary DICT_4X4_50 \
  --http-timeout 2.0
```

混合モデルがRoomba本体を誤検出する場合は実機試験を中止する。モデルをその場で変更しない。

## Phase A/B：物理出力なし

この段階ではPicoとROSの非ゼロロックを解除しない。

```bash
./simulation/sim.sh shell
ros2 topic info -v /roomba/cmd_vel_unstamped
ros2 topic echo /litter/target_decision
ros2 topic echo /nav/cmd_vel_unstamped
```

マスクなしで非ゼロNavigation指令がなく、マスク提示後に`accepted`、confidence 0.50以上、
新鮮なArUco姿勢による非ゼロNavigation指令を確認する。ここまで物理トピックのPublisherは0。

原因は `supervisord` です。`cmd-vel-output-adapter` が `autorestart=true` で管理されているため、PIDをkillしても再生成されます。

コンテナ内でプロセスではなくSupervisorサービスを止めてください。

```bash
supervisorctl stop cmd-vel-output-adapter
supervisorctl status cmd-vel-output-adapter
pgrep -af '[c]md-vel-output-adapter'
```

正常なら次の状態です。

```text
cmd-vel-output-adapter   STOPPED
```

`pgrep`は何も表示されません。

短縮E試験で実機用アダプターを使う場合は、この既定アダプターを停止したまま、必要なときだけ別ターミナルで1個だけ起動します。

```bash
cd /Users/imaichika/robot
./simulation/sim.sh cmd-adapter roomba /nav/cmd_vel_unstamped
```

注意点：

- この段階ではまだARMしないでください。
- `cmd-vel-output-adapter` が2個以上ないことを確認してください。
- コンテナを再起動するとSupervisorの `autostart=true` により再び起動します。
- 戻す場合は `supervisorctl start cmd-vel-output-adapter` ですが、実機用アダプターと同時起動しないでください。

つまり、これはプロセス漏れではなく、自動再起動設定どおりの挙動です。

## 実機試験直前の人間操作

1. Roombaを安定した台に置き、両輪を完全に浮かせる。
2. BRCを接続せず、CLEANボタンで手動起床する。
3. 独立停止操作と撮影を準備する。
4. Pico配備版のみ、既存の正式手順で`ALLOW_NONZERO_DRIVE=True`にして再起動する。
   Wi-Fi認証情報をリポジトリへコピーしない。
5. この時点ではARMしない。

STOP限定診断を実施する。

```bash
cd /Users/imaichika/robot
./simulation/sim.sh roomba-tcp-diagnostic 192.168.0.29 9000
```

`STOP confirmed ... ack=0x07`、right/left 0、`OI mode: 2 (Safe)`以外はNO-GO。
診断プロセスを`Ctrl+C`で終了してから次へ進む。

## Phase C：0.4秒ゲート付き物理出力

ターミナルAで短縮E専用ブリッジを起動する。これは接続診断を行うが、ARMしない。

```bash
cd /Users/imaichika/robot
TRIAL_ID="short_e_$(date +%Y%m%dT%H%M%S)"
mkdir -p "artifacts/short_e_wheel_lift/$TRIAL_ID"
./simulation/sim.sh roomba-short-e-bridge 192.168.0.29 WHEELS_LIFTED 9000 \
  2>&1 | tee "artifacts/short_e_wheel_lift/$TRIAL_ID/bridge.log"
```

この起動分岐は短縮Eブリッジを排他ロックし、同時に2個目を起動しようとすると
`Refusing duplicate roomba-short-e-bridge`（終了コード73）で拒否する。ホスト側は0.2秒周期の
ライフサイクル・ハートビートを送る。`Ctrl+C`や`tee`を含むパイプのEOFは即時に、
タイムアウトは0.5秒が2回連続した場合に、コンテナ側wrapperがブリッジへSIGINTを送り、
通常の終了時STOP処理を実行させる。単発の遅延はカメラ・AI負荷による誤終了を避けるため許容するが、
実走行停止を担うbridgeの500ms command timeoutとPicoの500ms watchdogは変更しない。

ターミナルAを`Ctrl+C`で終了した後は、再起動前にコンテナ内で残留がないことを確認する。

```bash
cd /Users/imaichika/robot
./simulation/sim.sh shell
pgrep -af '[r]oomba-bridge-tcp'
exit
```

正常時は`pgrep`が何も表示しない。表示が残る場合は次の試行を開始せずNO-GOとする。
この終了管理はlauncherの親子プロセスだけを対象とし、STOP、watchdog、ARM、TTLの判定値は
変更していない。

ターミナルBでNavigation出力アダプターを実機トピックへ接続する。ARM前の非ゼロはブリッジが拒否する。

```bash
cd /Users/imaichika/robot
./simulation/sim.sh cmd-adapter roomba /nav/cmd_vel_unstamped
```

ターミナルCでPublisherが1つだけであることを確認する。

```bash
cd /Users/imaichika/robot
./simulation/sim.sh shell
ros2 topic info -v /roomba/cmd_vel_unstamped
```

マスクを提示し、dry-run側で`accepted`と新鮮な非ゼロNavigation指令を確認してから、同じシェルで
一度だけ新規ARMを発行する。

```bash
GENERATION=$(date +%s)
ros2 topic pub --once /roomba/arm_generation std_msgs/msg/UInt32 \
  "{data: $GENERATION}"
```

最初の新鮮な非ゼロ指令から0.4秒でブリッジがSTOPを送りARMを破棄する。期待ログは次の順序。

```text
ARM accepted for this TCP session
bounded drive session started
bounded Drive ACK: status=0x03 ... right=... left=...
bounded drive deadline reached: STOP sent, ARM revoked
Rejected non-zero command: explicit ARM generation is required
```

左右値の絶対値が20以下でなければFAIL。期限ログと最終Drive ACKの時差が0.4秒を超えた場合もFAIL。
車輪が1秒以内に物理停止したことを動画または目視記録で確認する。

## Phase D：再発進防止

新規ARMを発行せず、マスクを隠して再提示する。非ゼロNavigation指令が残っても、
ブリッジは`explicit ARM generation is required`として拒否し、車輪が再回転しないことを確認する。

ターミナルBのアダプターを`Ctrl+C`で終了する。終了時STOPが送られる。次にターミナルAのブリッジも
`Ctrl+C`で終了する。順序を逆にしない。

## Phase E：終了診断とロック復帰

1. Pico配備版の`ALLOW_NONZERO_DRIVE=False`を復元して再起動する。
2. 実機出力アダプターが終了済みであることを確認する。
3. STOP限定診断を再実行する。

```bash
./simulation/sim.sh roomba-tcp-diagnostic 192.168.0.29 9000
```

ACK 0x07、right=0、left=0、Safe=2を確認後、診断を終了する。通常起動の
`drive_enabled=false`は変更しない。

## 即時中止条件

- Drive ACKが0x03以外（特に0x01）、ACK timeout、UART部分書込み。
- STOP診断が0x07・左右0・Safe=2を満たさない。
- ArUco姿勢鮮度切れ、古い検出、時計差による拒否。
- 0.4秒期限ログがない、非ゼロACKが期限後も続く。
- STOP後の再回転、想定外の回転方向・速度。
- Wi-Fi、電源、支持台、配線の異常。

中止時は成功扱いにしない。可能なら通常STOP、ブリッジ終了、独立停止の順に実行し、
Pico電源断やUART断だけで停止が保証されるとは扱わない。

## 記録と判定

保存先は`artifacts/short_e_wheel_lift/<TRIAL_ID>/`とする。

- `bridge.log`: ARM、Drive ACK、左右速度、期限STOP、拒否ログ。
- `result-E.json`: YOLO、ArUco、目標判断、Navigation、dry-run STOP。
- 動画: 最初の回転から物理停止、再提示時に再回転しない様子。
- `post-diagnostic.log`: 最終STOP ACK、左右0、Safe=2。

| 条件 | 結果 |
|---|---|
| マスクなしで非ゼロDriveなし | NOT_RUN |
| 3連続検出とaccepted | NOT_RUN |
| 実測ArUcoから非ゼロNavigation | NOT_RUN |
| 新規ARM後だけ車輪回転 | NOT_RUN |
| 左右輪20mm/s以下 | NOT_RUN |
| 非ゼロ許可0.4秒、物理回転1秒以内 | NOT_RUN |
| Drive ACK 0x03 | NOT_RUN |
| STOP後に物理停止 | NOT_RUN |
| ARMなし再発進なし | NOT_RUN |
| 最終ACK 0x07、左右0、Safe=2 | NOT_RUN |
| Pico/ROSロック復帰 | NOT_RUN |

全項目が実測PASSの場合のみ、次の有人限定床面AI試験を個別にGo判定する。

## テストと変更記録（2026-10-25）

今回追加したテストは次の2件で、実機、TCP 9000/9001、UARTへ接続しない。

- `test_short_e_ros_delay_or_silence_cannot_extend_fixed_deadline`
- `test_short_e_stop_latch_rejects_delayed_old_command_until_new_arm`

関連する期限・watchdogテスト5件はPASSした。上記2件を含む全回帰は295件PASSした。

計測関連の未コミット変更は次のとおり。

- 変更: `simulation/Dockerfile`
- 変更: `simulation/roomba_bridge_tcp.py`
- 変更: `simulation/roomba_tcp_transport.py`
- 変更: `simulation/vision_turn_controller.py`
- 変更: `tests/test_roomba_bridge_tcp.py`
- 変更: `tests/test_roomba_tcp_transport.py`
- 追加: `simulation/latency_trace.py`
- 追加: `simulation/navigation_latency_probe.py`
- 追加: `simulation/roomba_latency_probe.py`
- 追加: `simulation/yolo_inference_load.py`
- 追加: `tests/test_latency_trace.py`

本手順書も未コミット変更として整理対象に含める。計測成果物は`artifacts/`配下に保存され、
Gitの追跡対象外であるため、提出証跡として別途保全する。
