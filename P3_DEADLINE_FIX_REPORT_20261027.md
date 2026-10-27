# P3期限管理 集中修正報告（2026-10-27）

## 判定

- ソフトウェア模擬試験: **PASS**
- Pico実機への修正版配備: **未実施**
- Roomba実機での0.4秒遵守・UART STOP完了・物理停止: **未確認**
- P3実機走行: **NO-GOを維持**（下記の有人再試験がPASSするまで）

## 原因と遅延内訳

P2ログでは、セッション開始 `163453.037197` から期限検知ログ
`163453.480528` まで `0.443331 s`、すなわち `43.331 ms` 超過した。

主原因は `roomba_bridge_tcp.py` の20 Hz（50 ms周期）タイマーである。実測遅延は
その1周期以内に収まる。加えて、同じROS executorスレッドで同期実行される次の処理が
期限監視を遅らせ得た。

1. OI SafeのTCP/UART照会
2. TCP Drive送信
3. Drive ACK待ち
4. ログ／trace出力

旧ログの `STOP sent` はSTOP送信前に記録されていたため、期限検知時刻であり送信完了
時刻ではなかった。物理停止時刻を表すものでもない。

## 修正

### ホスト

- タイマー周期は既存の50 msを維持する。100 Hz化はアイドルSTOPとログを毎秒100回へ
  増幅したため撤回した。期限の正しさはタイマー周期に依存させない。
- 接続時または停止遷移時のSTOPは一度だけ送る。停止状態の20 Hz tickでは同一STOPを
  再送せず、Picoの`normal STOP forwarded`ログとTCP負荷を増幅しない。
- Safe照会後、Drive dispatch直前に固定期限を再検査する。
- transport層でもsocket送信直前に絶対monotonic期限を検査し、期限以上なら非ゼロDriveを
  送らず `DriveDeadlineExpired` とする。
- TCP送信時間とDrive ACK待ち時間を別々にtraceへ記録する。
- 期限STOPについてだけ次を別イベントとして記録する。通常のアイドルSTOPでは出力しない。
  - `bounded drive deadline detected`: ホスト期限検知
  - `stop_tcp_send_begin`: TCP STOP送信開始
  - `stop_tcp_send_return`: OSへのTCP送信要求の復帰
- TCP送信復帰はPico UART完了でも物理停止でもないことをログに明記した。

### Pico

- `DRIVE_SESSION_TIMEOUT_MS = 400` を追加した。
- 最初にUARTへ正常転送された非ゼロDriveを起点にし、後続Driveで更新しない。
- 400 ms到達後は次のDriveをUART書込み前に拒否する。
- receive loopでも独立期限を監視し、STOPをUARTへ完全書込みしてSTOPラッチする。
- 期限検知とUART STOP完了を別々に出力する。
- 既存500 ms command watchdog、ARM世代／接続ID、STOPラッチ、速度上限は変更していない。

Picoのループは5 ms pollingであり、UART処理やACK送信中のスケジューリング遅延は実機ログで
測る必要がある。そのため「400 msで物理停止済み」とはまだ判定しない。

## 模擬・回帰試験

対象116件を実行し **116/116 PASS**:

```text
python3 -m unittest \
  tests.test_roomba_tcp_transport \
  tests.test_roomba_bridge_tcp \
  tests.test_pico_relay_protocol \
  tests.test_pico_relay \
  tests.test_differential_drive

Ran 116 tests in 0.216s
OK
```

追加確認項目:

- 0.399999秒ではdispatch許可、0.400000秒では拒否
- ROS timerが動かなくてもtransport直前ガードが期限後Driveをsocketへ渡さない
- Pico独立期限でUART STOPを書き、STOPラッチする
- Pico期限後の次DriveをUARTへ書かない
- 500 ms watchdog既存試験PASS
- ACK部分送信、ARM世代、再接続、STOP、20 mm/s速度変換の回帰PASS

全テスト自動探索320件では今回と無関係なローカル依存不足（`numpy`, `cv2`）および既存の
`test_ip` import名衝突により11件が収集エラーとなった。期限関連対象116件には失敗なし。

## 次の有人最小試験（1件のみ）

修正版 `pico/main.py` を配備し、安全ゲートを再確認したうえで、両輪浮上状態における
20 mm/s・最大0.4秒の単発試験を1回だけ行う。ホストtraceの期限検知／STOP送信開始・復帰、
Picoの期限検知／UART STOP完了、動画での物理停止を同一試行で保存する。期限後Driveが1件でも
あればFAILとし、その試行では再ARM・再Driveしない。
