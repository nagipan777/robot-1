# 有人限定床面試験 Go / No-Go Checklist

更新日: 2026-10-22。この表は、監視下で行う20 mm/s・1秒以下の単発直進だけに適用する。
無人走行、Navigation、YOLO追従、連続指令、旋回には適用しない。

## 提出用の固定構成

```text
MacBook -- Wi-Fi / TCP 9000 --> Pico W -- UART1 (GP4/GP5) --> Roomba 680
                                      ^
                                      |
                             モバイルバッテリー USB 5 V
```

Pico WとMacBookはUSB接続しない。制御・診断には既存のTCP 9000 ACK、ROS2 bridgeログ、
終了後のSTOP限定診断を使用し、UDP診断は追加しない。Pico内部のUSB REPLログは補助証跡であり、
有人限定床面試験の必須条件にはしない。

BRC（Roomba pin 5 / Pico GP2）は物理的に接続しない。RoombaはCLEANボタンで手動起床し、
起床後にSTOP限定診断を実行する。BRC自動起床は提出後課題とする。

## 判定の分離

| 試験 | 現在の判定 | Goに必要な条件 |
|---|---|---|
| STOP限定診断 | **GO** | BRCなし、手動起床、ACK 0x07、左右0、OI Safe=2を試行ごとに確認 |
| 有人・限定床面直進 | 条件付きHOLD | 下表が全てPASS、試験責任者が残留リスクを受容 |
| 無人／自律走行 | **NO-GO** | 電源断・UART断を含む独立停止、実姿勢、床面制御の別検証 |

## 有人試験の必須条件

空欄、`FAIL`、`UNKNOWN`が1つでもあればNo-Go。

| 項目 | 状態 | 当日の証拠 |
|---|---|---|
| Phase 4B回転・通常STOP・物理停止 | PASS（前回）/再確認要 | 試験ID・ログ |
| BRC未接続、CLEANによる手動起床 | PASS（2026-10-22）/当日再確認要 | 配線写真・操作記録 |
| STOP ACK 0x07、要求左右0、OI mode 2 | PASS（2026-10-22）/当日再確認要 | bridgeログ |
| `ALLOW_NONZERO_DRIVE=False`から開始 | UNKNOWN | Pico起動ログ／配備ファイル |
| `drive_enabled=false`から開始 | PASS（既定）/当日再確認要 | bridge起動引数 |
| PicoはモバイルバッテリーUSB 5 Vのみで給電し、Mac USBは未接続 | UNKNOWN | 配線写真 |
| 非走行状態で、BRCなし構成の給電・Wi-Fi安定性を確認 | PASS（2026-10-22）/当日再確認要 | pingログ、再起動/IP変化の有無 |
| UART1 GP4 TX/GP5 RX、共通GND、レベル変換、発熱・膨張に異常なし | UNKNOWN | 目視・写真 |
| wheel drop、cliff、バンパーを無効化していない | PASS（コード）/当日確認要 | Safe診断 |
| 充電器・Home Baseから分離 | UNKNOWN | 目視 |
| 平坦な閉鎖区域、人・ペット・ケーブル・段差なし | UNKNOWN | 全景写真 |
| 全周に剛性のある物理境界がある | UNKNOWN | 寸法・写真 |
| 進行方向に十分な停止余裕がある | UNKNOWN | 実測値 |
| 操作者1名がRoombaと端末を常時監視 | UNKNOWN | 氏名・時刻 |
| 操作者がCLEAN/電源操作へ即時に到達可能 | UNKNOWN | ドライラン |
| 物理境界でSTOP不能時も区域外へ出ない | UNKNOWN | 手押し確認 |
| 初回は左右20 mm/s、1秒以下、理論最大2 cm | UNKNOWN | 起動引数・ログ |
| BRCなし現構成で全非ゼロsequenceのDrive ACKが0x03 | UNKNOWN（UART部分書込みリスク） | 車輪浮上回帰ログ |
| Navigation、YOLO、他のcmd_vel Publisher停止 | UNKNOWN | `ros2 topic info -v` |
| 試験後STOP、切断ラッチ、両ロック復帰を確認 | UNKNOWN | 終了ログ |

## 非走行のモバイルバッテリー事前確認

走行ロックを解除せず、Roombaへ非ゼロ指令を送らない状態で行う。

1. `ALLOW_NONZERO_DRIVE=False`、`drive_enabled=false`を確認する。
2. MacBookとのUSBケーブルを外し、Pico WをモバイルバッテリーのUSB 5 Vだけで起動する。
3. UART1がGP4/GP5、GND共通、適切なレベル変換経由であることを目視確認する。
4. 最終デモ予定時間以上、Wi-Fi到達性、IP維持、意図しない再起動、給電停止、異常発熱を監視する。
5. 最後にSTOP限定診断を人が明示的に実行し、ACK 0x07、要求左右0、OI mode 2を確認する。

途中で給電停止、IP変更、再起動、連続した通信断、異常発熱があればFAILとし、床面試験へ進まない。
Pico内部ログを取得できないことだけをFAIL理由にはしないが、TCP ACKまたは終了後診断を取得できない場合は
No-Goとする。

## STOP不能時の補償策

Pico電源断またはPico TX–Roomba RX断では、新しいDrive Directゼロを送れない。走行中に電源や
UARTを抜いて試験してはならない。有人限定試験では次を同時に使う。

1. 全周を剛性のある境界で閉じ、階段、落下、道路、人、動物へ到達不能にする。
2. 最初の試行は障害物のない直線、左右20 mm/s、最大1秒、単一ARMだけにする。
3. 操作者は区域外から監視し、走行中に車体、車輪、配線へ手を入れない。
4. 通常STOPに反応しない場合は追加指令を送らず、CLEAN/電源操作を試みる。
5. それでも動く場合は物理境界内に封じ込め、人を近づけず、停止後に電源・配線を処置する。
6. 境界を越える可能性、異音、発熱、通信異常があれば即FAILとし、再ARMしない。

この対策は独立した安全認証済みE-stopではなく、小規模な有人試験の封じ込めである。その限界を
試験責任者が受容できない場合はNo-Goとする。

## 一回限りの許可

- 開始時は両ロックfalse。
- 操作者がPicoロックを手動で解除し、ROS bridgeも当該試験だけ`drive_enabled=true`で起動する。
- 新しいTCP sessionとARM generationを1回発行する。
- テストノードは1秒以内に通常STOPをPublishする。
- Picoの通常STOPは`ARMED -> STOPPED`、切断は`LATCHED_STOP`へ遷移する。
- bridge終了でSafe gate、ARM、古いTwistを破棄する。
- 最後にPico定数をfalseへ戻して再配備し、STOP限定診断を行う。

通常STOPや再接続だけで旧ARMを再使用してはならない。

## 承認後の最小実行手順

この節はチェック表が全てPASSになった後だけ使用する。まず変更をコンテナへ反映する。

```bash
cd /Users/imaichika/robot
./simulation/sim.sh up
```

通常設定のSTOP限定診断でACK 0x07、左右0、Safe=2を確認して終了する。次に操作者がPicoの
`ALLOW_NONZERO_DRIVE=True`を当該試験のためだけに手動配備する。認証情報は編集・記録しない。

ターミナルAで、既存bridgeを20 mm/s上限・通常watchdogで起動する。

```bash
./simulation/sim.sh shell
source /opt/create3_ws/install/setup.bash
/usr/local/bin/roomba-bridge-tcp --ros-args \
  -p host:=PICO_W_IP -p port:=9000 \
  -p drive_enabled:=true -p diagnostic_on_connect:=true \
  -p watchdog_test_mode:=false -p max_speed_mm_s:=20 \
  -p command_timeout:=0.5 \
  -p command_topic:=/roomba/cmd_vel_unstamped
```

事前診断合格後、ターミナルBで既存コンテナへ入り、一度だけ実行する。

```bash
cd /Users/imaichika/robot
./simulation/sim.sh shell
source /opt/create3_ws/install/setup.bash
/usr/local/bin/roomba-wheel-lift-test --ros-args \
  -p mode:=floor_straight \
  -p allow_rotation:=true \
  -p confirmation:=ATTENDED_FLOOR_CLEARED \
  -p arm_generation:=$(date +%s) \
  -p left_mm_s:=20.0 -p right_mm_s:=20.0 \
  -p drive_duration_s:=1.0
```

このコマンドは左右+20 mm/s、最大1秒、単一ARM generationに固定される。終了STOPによりPicoは
`STOPPED`へ戻る。ターミナルAを`Ctrl+C`で終了すると切断STOPと`LATCHED_STOP`が追加される。

## 終了後

1. 物理停止と意図しない再始動なしを目視記録する。
2. TCP ACK、ROS2 bridgeログ、通常STOP、切断時の停止結果を保存する。Pico内部ログは取得できる場合だけ補助証跡とする。
3. `ALLOW_NONZERO_DRIVE=False`へ戻して手動再配備する。
4. `roomba-tcp-diagnostic`を`drive_enabled=false`で再実行する。
5. ACK 0x07、要求左右0、Safe=2、STOP-onlyを確認する。
6. JSON/CSVへ実測距離、encoder差分、停止理由、PASS/FAILを記録する。
