# 60--100 mm 有人連続直進試験手順書

## 適用範囲

本書は試験計画であり、現時点では実機未検証である。実行にはPico配備差分の照合、
非走行診断、試行責任者による段階ごとの個別承認が必要となる。

## 共通前提

- 単一`roomba-trial-session`だけがTCPを所有する。
- BRC未接続、CLEAN手動起床、独立停止手段準備済み。
- STOP ACK `0x07`、要求right/left `0/0`、OI Safe 2。
- `COMMAND_TIMEOUT_MS=500`と20 mm/s上限は不変。
- `CONTINUOUS_DRIVE_TEST_ENABLED=True`は承認済み試行中だけ使用する。
- host秒とPicoミリ秒の固定期限が完全一致する。
- ARM generationは未使用値で、接続後に1回だけ明示送信する。
- STOP後、自動再ARM・Drive再送をしない。

## 試行段階

| trial | speed | host | Pico | theoretical | 開始条件 |
|---|---:|---:|---:|---:|---|
| C1 | 20 mm/s | 1.0 s | 1000 ms | 20 mm | 浮上、非走行診断PASS |
| C2 | 20 mm/s | 2.0 s | 2000 ms | 40 mm | C1を3回PASS |
| C3 | 20 mm/s | 3.0 s | 3000 ms | 60 mm | C2を3回PASS、床面経路クリア |
| C4 | 20 mm/s | 5.0 s | 5000 ms | 100 mm | C3を3回PASS、距離再現性確認 |

時間だけを変更し、速度と制御周期を同時に変更しない。

## 各試行の手順

1. 適用commit、Pico配備ファイルSHA256、4つの設定値を記録する。
2. Roombaを支持台へ置き、両輪浮上、配線、独立停止手段を確認する。
3. Picoを`ALLOW_NONZERO_DRIVE=False`で起動し、単一bridgeで起動診断する。
4. STOP ACK `0x07`、0/0、Safe 2、bridge/ARM subscriber/command subscriber各1を記録する。
5. host/Pico期限、20 mm/s、500 ms watchdog、trace/CSV出力先を照合する。
6. 当該trialの個別承認後だけPicoロックを解除し、再起動・静止を再確認する。
7. 未使用generationでARMし、同世代のsequenceを100 ms周期の候補として送る。
8. 固定期限または人間STOPの早い方で終了する。
9. STOP ACK、Pico UART STOP、物理停止、5秒間再発進なしを確認する。
10. bridgeを終了し、Picoを`ALLOW_NONZERO_DRIVE=False`へ戻す。
11. STOP-only診断後、USBを外して停止を確認する。

実行コマンドは専用launcher実装・非走行検証・配備承認後に確定する。既存400 ms launcherの
引数を手作業で書き換えて実行してはならない。

## 記録

- ARM generation、TCP connection ID
- Drive送受信時刻、sequence、ACK status
- 最大/p95 Drive間隔、ACK遅延
- host deadline/STOP begin/return
- Pico fixed-deadlineまたはwatchdog発火時刻
- Pico UART STOP complete
- STOP ACK、物理停止、再発進
- ArUco開始/終了x, y, yaw、欠損、外れ値
- 理論距離、実測距離、誤差、横ずれ

## PASS

- generation/connection IDが全Driveで一致。
- sequenceが1から単調増加し、ACK status `0x03`。
- Drive間隔が500 ms未満。
- 速度指令が左右とも0～20 mm/sの同符号。
- 設定固定期限までにSTOP処理が始まり、Pico UART STOPを確認。
- 物理停止後5秒間再発進なし。
- 床面試験ではArUco変位が前方で、測定分解能を上回る。

## FAIL・中止

- ACK timeout/部分応答、不正CRC/status、generation/sequence不一致。
- TCP断、Safe喪失、500 ms watchdog発火、設定期限不一致。
- 逆転、意図しない旋回、異音、配線/支持/経路異常。
- STOP後のDriveまたは再発進。
- ArUco欠損は通信試験FAILではなく距離評価INVALID。ただしC3/C4移行条件は未達。

失敗試行では再ARMしない。次の段階へ自動移行せず、ログから故障モードを確定する。
