# Roomba単一ブリッジ運用 非走行比較

日付: 2026-10-29  
対象: `fix-arm-ack-session-recovery`

## 目的と変更範囲

STOP専用ブリッジを終了して走行用ブリッジを起動する運用をやめ、既存
`roomba_bridge_tcp.py`の接続時STOP・Safe診断、ARM、deadline STOP、再接続診断を
同じプロセスで使う。UART/TCP protocol、Pico、P3-2 deadline処理は変更しない。

## 旧手順との比較

| 項目 | 旧手順 | 新手順 `roomba-trial-session` |
|---|---|---|
| bridge起動 | STOP診断用を起動・終了後、走行用を起動 | 走行可能だが未ARMのbridgeを1回だけ起動 |
| 起動時診断 | STOP専用と走行用で重複 | 既存`diagnostic_on_connect`を1回利用 |
| 定常bridge数 | 切替時に重複・孤児化の余地 | 1。共有`flock`と`/proc`検査で2個目を拒否 |
| ARM前 | 手操作中に候補が古くなる可能性 | 既存`attended-nav-arm-gate`で新鮮候補2件を確認 |
| STOP後 | bridgeを切り替える場合がある | 同じbridgeでSTOP。ARMは失効し自動再ARMなし |
| 再接続 | 新プロセス起動に依存 | 同じbridgeの既存`connect()`がSTOP/Safeを再診断 |
| ログ | terminalごとに手動保存 | ROS logとlatency JSONLを試行固有directoryへ保存 |

## 排他と孤児検出

実機用の`roomba-tcp`、`roomba-tcp-diagnostic`、短縮E、有人床面、統合sessionは
`/tmp/roomba-physical-bridge.lock`を共有する。lock取得失敗はexit 73で走行拒否する。
古いランチャーや異常終了でlockを所有しない`roomba-bridge-tcp`が残った場合も、
`/proc/*/cmdline`検査で検出してexit 74とする。既存プロセスを自動killしないため、
対象を確認せず別試行を開始しない。

## 新しい有人運用

全安全ゲートと当該試行の個別承認は従来どおり必要である。

1. 次を1回だけ起動する。

   ```bash
   ./simulation/sim.sh roomba-trial-session PICO_IP ATTENDED_FLOOR
   ```

2. 同じbridgeがREADY、STOP ACK `0x07`、左右0、OI Safe 2を確認してARM待機する。
3. controller/ArUco状態を新しい試行として初期化し、有人承認後だけ
   `attended-nav-arm-gate`を明示generationで起動する。
4. gateは0.25秒以内の非ゼロNavigation候補2件、ARM subscriber 1件の場合だけARMを
   1回publishする。候補なし・古い姿勢・重複subscriberではARMしない。
5. 20 mm/s以下・400 ms以下の既存bridge/Pico制限で単発試験し、STOP後は再ARMしない。
6. `Ctrl+C`で同じbridgeを終了する。shutdown STOP後、試行固有のROS logと
   `latency.jsonl`を`artifacts/roomba_trial_sessions/`へ保存する。

## 非走行検証

- `bash -n simulation/sim.sh`: PASS。
- bridge focused tests: 24/24 PASS（変更直後）。
- 全回帰: 381/381 PASS。
- 単一session branch内の`roomba-bridge-tcp`起動箇所: 1。
- STOP専用・診断・有人launcherが同じlock名を使用: PASS。
- duplicate拒否、lockを持たないorphan拒否、試行固有log path: source test PASS。
- `roomba_bridge_tcp.py`、`roomba_tcp_transport.py`、Pico codeの差分: なし。
- 実Pico/Roombaでのsession起動・再接続・ログ回収: 未確認。

## 判定

非走行の構成・回帰: **PASS**。新運用はブリッジ切替をなくし、定常bridgeを1個に
限定する。実機運用PASSではないため、次回はまずlockを維持したSTOP/Safe診断とログ
回収だけを確認し、その後に別途承認された単発試験へ進む。
