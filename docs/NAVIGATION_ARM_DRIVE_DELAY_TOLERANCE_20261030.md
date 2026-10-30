# Navigation ARM・Drive通信遅延耐性 非走行検証

更新日: 2026-10-30
対象: `feature/navigation-hardware-unit-test`
区分: 非走行・模擬TCP。Pico/Roomba未接続、ARM/Drive実送信なし。

## 1. 結論

現行プロトコルは、一度ACK済みのARM generationを同一TCPセッションで複数の
sequenced Driveに利用できる。正常なDrive ACKが400 msの固定セッション期限より前に
完全検証されれば、50--300 msの遅延でもARMを維持し、sequenceを1つだけ進める。

ACKが期限までに確定しない場合は、RoombaがDriveを実行したかホストから確定できない。
この場合は再送せず、同じソケットでSTOPを試行して接続を閉じ、ARMとsequenceを破棄する。
これは不要な失効ではなく、二重実行を防ぐために必要な失効である。

## 2. 現在のARM失効原因

| 原因 | 故障モード | 現行動作 | 分類 |
|---|---|---|---|
| 正常STOP | 停止後に古い指令が再生される | ARM、sequence、sessionを失効 | 必要 |
| 400 ms固定期限 | 単発試験範囲を越えてDriveが継続 | STOP、接続状態に応じARM失効 | 必要 |
| 500 ms Pico watchdog | ホスト通信停止中もUART速度が残る | PicoがSTOP latch | 必要 |
| ACK timeout/部分受信 | Drive実行済みか不明 | STOP、切断、再送禁止 | 必要 |
| ACK CRC/type/status/generation/sequence不正 | 別応答の誤採用、世代混線 | STOP、切断、再同期 | 必要 |
| Safe mode喪失 | OIがSafe以外で動作 | STOP、許可取消 | 必要 |
| TCP切断/再接続 | 旧セッションのARM再利用 | disconnect STOP、再診断 | 必要 |
| ARM後に最初のDriveが来ない | Pico側ARMの放置 | 既存500 msでARM expiry | 必要 |

正常ACK遅延そのものはARM失効原因ではない。ただし400 ms期限を跨いだACKは正常な
継続条件として採用しない。

## 3. 状態の分離

今回、既存プロトコルを変更せず次の状態を明示した。

1. **ARM validity**: `armed_generation`。STOP、切断、異常でのみ破棄。
2. **Drive ACK**: `DriveAckState`の`idle/pending/confirmed/failed`、generation、sequence、経過時間。
3. **Navigation command freshness**: `CommandState`の最新値、受信monotonic時刻、revision。
4. **fixed drive deadline**: `DriveSessionLimiter`の開始時刻と400 ms絶対期限。
5. **Safe state**: STOP ACK 0x07、要求0/0、OI Safe 2からなる既存`SafeModeGate`。

ACK pending中に別のDriveを開始すると`DriveAckState`が拒否する。正常ACK確定時だけ
`drive_sequence`を進める。ROS QoSはKEEP_LAST/depth=1であり、未処理Navigation候補を
複数蓄積しない。

## 4. Trace追加

正常時:

- `tcp_drive_ack_wait_start`: generation、sequence、固定期限
- `tcp_drive_ack_received`: raw、待機時間
- `tcp_drive_ack_validated`: generation、sequence、status、ACK全経過時間

異常時:

- `tcp_drive_ack_failed`: generation、sequence、受信byte数、期待byte数、経過時間、理由
- 期限到達時は既存`tcp_deadline_stop_begin/return`を併記

これにより、0/13 byte timeoutと1--12/13 byte部分応答を、正常遅延から区別できる。

## 5. 模擬遅延結果

共通条件: generation 71、sequence 1、20 mm/s、固定期限400 ms、Pico/Roombaなし。

| ACK遅延 | ACK状態 | ARM | sequence | STOP | 二重Drive |
|---:|---|---|---:|---|---:|
| 0 ms | confirmed | 維持 | 1 | なし | 0 |
| 50 ms | confirmed | 維持 | 1 | なし | 0 |
| 100 ms | confirmed | 維持 | 1 | なし | 0 |
| 200 ms | confirmed | 維持 | 1 | なし | 0 |
| 300 ms | confirmed | 維持 | 1 | なし | 0 |
| 600 ms相当 | fixed deadlineでfailed | 失効 | 0へ破棄 | 400 msで開始 | 0 |
| 1/13 byte後に期限 | failed | 失効 | 0へ破棄 | 期限で開始 | 0 |

300 ms ACKは通信上は正常だが、残りの固定走行枠は約100 msしかない。ACK確定後に
新しいDriveを出せる条件は、ARM有効、最新指令が500 ms以内、上流の姿勢・目標が新鮮、
Safe維持、かつ400 ms絶対期限より前である。いずれかが満たせなければSTOPする。

## 6. 既知の制約

`Twist`はunstampedで、現行bridgeはsingle-threaded executor内で同期TCP ACKを待つ。
このためACK待機中にDDSへ到着した候補はdepth=1で最新1件に圧縮されるが、bridgeの
callbackが実行されるまで実到着時刻を取得できない。したがってbridgeの
`CommandState.received_at`だけでは、その候補の画像・姿勢・目標の生成時刻を証明できない。

今回、これを解消するためのMultiThreadedExecutor化や通信worker化は行っていない。
STOPとDriveが別threadから同じsocketを操作すると応答混線を起こし得るためである。
連続閉ループへ進む前には、次のどちらかが必要である。

- stampedなNavigation候補に生成時刻・姿勢時刻・目標時刻を含め、送信直前に検査する。
- 単一socket所有のI/O workerを設け、STOP優先queueと最新1件command slotを実装する。

これはPhase 1静止診断のブロッカーではないが、400 msを越える連続Navigation試験の
ブロッカーである。

## 7. 長時間直進モードへの適用条件

通常の400 msモードを延長して流用してはならない。別の明示的モードとして、最低限次を
非走行・浮上・床面の順に検証する。

1. stamped候補の生成時刻、姿勢時刻、目標時刻を送信直前に検査できる。
2. socket所有者が1つで、STOPが未送信Driveを破棄できる。
3. ACK pending中はDrive再送なし、最新候補は1件だけ。
4. ACK確定後も500 ms watchdogより短い更新周期を実測で維持する。
5. 入力失効、ACK不明、Safe喪失、通信断でSTOPし、自動再ARMしない。
6. 400 msモードと設定・起動コマンド・ログを明確に分離する。
7. 速度、時間、旋回範囲は一変数ずつ拡張し、各段階を試験責任者が承認する。

## 8. 判定

- **PASS**: ARM・Drive・Pico重点回帰114/114、全回帰408/408。
- **PASS**: 正常ACK 0--300 msでARM維持、sequence連続、二重送信なし。
- **PASS**: timeout/部分応答でSTOP、接続poison、ARM失効、再送なし。
- **PASS**: 400 ms上限、500 ms watchdog、STOP latch、単一TCP所有を変更していない。
- **UNVERIFIED**: ROS実行中にACK待機と同時到着した候補の生成時刻評価。
- **UNVERIFIED**: 実Wi-Fi/Picoでの遅延分布とSTOP物理時刻。
- **BLOCKED**: 1秒超の連続走行。専用モードの設計・非走行検証・個別承認が必要。
