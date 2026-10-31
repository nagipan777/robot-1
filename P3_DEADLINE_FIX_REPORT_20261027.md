# P3期限管理 集中修正報告（2026-10-27）

## 判定

- ソフトウェア模擬試験: **PASS**
- Pico実機への修正版配備: **PASS**
- Roomba実機での0.4秒遵守・STOP・物理停止: **PASS**
- P3期限ブロッカー: **解除**
- AI連動P3全体: 別途YOLO・ArUco・Navigationの試行固有ゲート確認が必要

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

Picoの独立期限そのものを意図的に発火させる異常系実機試験は未実施である。今回の正常系では
ホストPublisherのSTOPが0.4秒より先に到達した。独立期限の境界・UART STOP・ラッチはPico
モックで検証し、実機のバックアップ条件として維持する。

## 模擬・回帰試験

重複STOP抑止と非走行dry-runを含む対象119件を実行し **119/119 PASS**:

```text
python3 -m unittest \
  tests.test_roomba_tcp_transport \
  tests.test_roomba_bridge_tcp \
  tests.test_pico_relay_protocol \
  tests.test_pico_relay \
  tests.test_differential_drive \
  tests.test_deadline_dry_run

Ran 119 tests in 0.706s
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
`test_ip` import名衝突により11件が収集エラーとなった。期限関連対象119件には失敗なし。

## 非走行E2E

`simulation/deadline_dry_run.py`を実行し、意図的な約38.6 msのexecutor遅延後も期限後の
socket送信が0件であることを確認した。ARMと実機Driveは送信していない。成果物は
`artifacts/deadline_dryrun/20261027/`へ保存した。

## ログ増幅インシデントと修正

初回修正でタイマーを100 Hzへ変更し、共通`send_stop()`へ毎回INFOログを追加したため、
アイドル状態でもSTOPとログが最大100回/秒発生した。物理的な非ゼロDriveではなかったが、
TCP負荷と停止操作の視認性を悪化させるため使用禁止とした。

100 Hz化を撤回し、期限ログを期限STOPだけへ限定した。さらに停止中の同一STOPを一度だけに
抑止した。修正後の実通信STOP限定診断では、READY、ACK `0x07`、左右0、Safe 2の表示後に
ログが静止した。

## 有人両輪浮上E2E

重複STOP修正版を使用し、左右20 mm/s・最大0.4秒の単発試験を実施した。

- ARM generation: `1793101962`
- Drive ACK: `0x03`、sequence 1〜3
- 左右要求速度: 20 / 20 mm/s
- セッション開始: `169453.620066`
- 正常STOP受信・送信: `169454.010324`
- 経過: **0.390258秒**（0.4秒以内）
- 期限後Drive: 0件
- 人間の目視: 両輪停止、異音なし、再発進なし

終了後に`ALLOW_NONZERO_DRIVE=False`へ戻して再起動し、STOP ACK `0x07`、左右0、OI Safe 2、
ログ静止を再確認した。ブリッジを終了し、Pico USBを外し、LED消灯と両輪停止を確認した。
