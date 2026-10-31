# C1連続直進・両輪浮上試験結果

日付: 2026-10-30  
条件: 左右20 mm/s、host 1.0 s、Pico 1000 ms、500 ms watchdog

## 試行1

判定: **PASS**

- 支持: 両輪完全浮上、支持安定、配線干渉なし
- ARM generation: `1793344989`
- pre-ARM session probe: STOP `0x07`、左右0、Safe 2
- Drive ACK: 13/13、sequence 1--13、全status `0x03`、right/left `20/20`
- 最大Drive送信間隔: `126.809 ms`（500 ms未満）
- p95 Drive送信間隔: `126.809 ms`（12標本nearest-rank）
- 最大ACK待ち: `125.550 ms`
- p95 ACK待ち: `125.550 ms`（13標本nearest-rank）
- host deadline検知遅れ: `0.832 ms`
- deadline STOP TCP送信: `0.634 ms`
- deadline後Drive: 0件
- 自動再ARM: 0件
- 物理確認: 約1秒回転後、両輪停止、異音・支持ずれなし、5秒以上再回転なし
- 終了後: continuous mode False、nonzero lock Falseで再起動
- 終了診断: STOP `0x07`、左右0、OI Safe 2

deadline到達時はlive OI mode照会を中断し、同一socket所有者がSTOPを送ってsessionを
失効した。ログ上の走行許可取消はACK異常ではなく、固定期限による期待どおりの停止である。

証拠:

- `artifacts/roomba_trial_sessions/85267-1793344916-19096/latency.jsonl`
- latency SHA256: `5d77b40353232eba2b8c2dea401ffc9c1ef2568b8788401aa9896bea349bd59c`
- ROS log SHA256: `a98c05757f403caa95d790a5d014760fbba769b5736df32f39f1811155c5d0ec`

## 進捗

- C1 PASS: 1/3
- C2開始条件: 未達（C1をあと2回PASSする必要あり）

## 試行2（C1回数には不算入）

判定: **INVALID（安全上のFAILではない）**

- Drive ACKとSTOPは成立し、物理停止、支持安定、異音なし、再回転なしを確認
- 非ゼロpublish窓は約0.90秒で、要求した1.0秒より早く終了
- 原因: ARM settle終了を計時起点にしており、0.1秒周期の最初の非ゼロpublishまでの待ちも走行時間へ算入していた
- 対応: 計時起点を最初の非ゼロpublishへ変更

安全な早期停止であるためC1失敗とはしないが、固定期限の検証標本には採用しない。
修正後もC1進捗は1/3であり、個別承認された有効試行があと2回必要である。

## 修正版試行2（Pico期限証拠なし）

判定: **UNVERIFIED（C1回数には不算入）**

- ARM generation: `1793347662`
- 最初の非ゼロpublishからSTOP publish: 約`1.000849 s`
- 確定Drive ACK: sequence 1--10、全status `0x03`、right/left `20/20`
- sequence 11: 期限前に送信したがACK待機中にhost期限へ到達し、0/13 byteのまま未確定
- host期限検知遅れ: `1.855 ms`
- deadline STOP TCP送信: `0.267 ms`
- host期限後Drive送信: 0件
- 再送・自動再ARM: 0件
- 再接続後: STOP `0x07`、左右0、OI Safe 2
- 物理確認: 両輪停止、異音・支持ずれなし、5秒以上再回転なし
- 終了後: continuous mode False、nonzero lock Falseで再起動

Picoログはロック復帰の再起動前に保存されず、独立1000 ms期限とUART STOP完了の
試行中証拠を回収できなかった。安全成功を否定するログはないが、Pico独立期限を推測で
PASSにしないためC1標本には採用しない。次回は物理停止確認の直後、Pico再起動より先に
ARM generationから最初のSTOPまでのログを保存する。

証拠:

- `artifacts/roomba_trial_sessions/9126-1793347479-15503/latency.jsonl`

進捗は引き続き **C1 PASS 1/3**。

## ARM受理同期前の次試行（C1回数には不算入）

判定: **INVALID（安全上のFAILではない）**

- ARM generation: `1793348457`
- Drive ACK: sequence 1--12、全status `0x03`、right/left `20/20`
- host session開始から通常STOP: 約`0.619 s`
- Pico最初のDriveから通常STOP: `529 ms`
- Pico watchdog STOP: 最終Driveから`501 ms`、UART書込み成功
- 物理確認: 両輪停止、異音・支持ずれなし、5秒以上再回転なし

publisherは固定settle後に非ゼロpublishを開始したが、pre-ARM STOP/Safe診断とPico
ARM ACK検証の完了を知らなかった。そのためpublisherの1秒窓の一部がARM確定前に
消費され、host/Picoの実Drive窓が短くなった。

ブリッジがPico ARM ACK検証後に`/roomba/arm_accepted_generation`へ受理generationを
publishし、試験publisherは同一generationを受信するまで非ゼロをpublishしないように
修正した。通知がない場合は5秒後にSTOPのみで終了する。Picoプロトコル、速度上限、
1000 ms独立期限、500 ms watchdog、STOPラッチは変更していない。

非走行実通信（Pico lock False）ではARM status `0x02`拒否後、受理通知なし、非ゼロ0件、
5秒timeout、STOPのみ、再接続後STOP `0x07`・左右0・Safe 2を確認した。

証拠:

- `artifacts/roomba_trial_sessions/16412-1793348393-5263/latency.jsonl`
- `artifacts/roomba_trial_sessions/20865-1793348858-11318/latency.jsonl`

## ARM受理同期修正版・有効試行

判定: **PASS**

- ARM generation: `1793349121`
- ARM受理通知前の非ゼロpublish: 0件
- Drive ACK: sequence 1--17、全status `0x03`、right/left `20/20`
- host期限検知遅れ: `1.775 ms`
- deadline STOP TCP送信: `0.138 ms`
- host期限後Drive: 0件
- Pico最初のDrive: `6739668 ms`
- Pico最終Drive: sequence 17、`6740512 ms`
- Pico最初の通常STOP: `6740532 ms`
- Pico実DriveからSTOP: 約`864 ms`（1000 ms上限以内）
- watchdog冗長STOP: 最終Driveから`504 ms`、UART書込み成功
- 自動再ARM・Drive再送: 0件
- 物理確認: 両輪停止、異音・支持ずれなし、5秒以上再回転なし

host固定期限STOPがPico独立1000 ms期限より先に到達したため、Pico session deadlineは
発火しなかった。これは最大時間内のhost正常停止であり、Pico独立期限の不成立ではない。
Pico独立期限は別の既存試行で確認済みである。

証拠:

- `artifacts/roomba_trial_sessions/22758-1793349069-68/latency.jsonl`

進捗: **C1 PASS 2/3**。C2開始条件には有効C1 PASSがあと1回必要。

## ARM受理同期修正版・最終有効試行

判定: **PASS**

- ARM generation: `1793349637`
- ARM受理通知前の非ゼロpublish: 0件
- Drive ACK: sequence 1--13、全status `0x03`、right/left `20/20`
- host期限検知遅れ: `0.593 ms`
- deadline STOP TCP送信: `0.101 ms`
- host期限後Drive: 0件
- Pico最初のDrive: `7255909 ms`
- Pico最終Drive: sequence 13、`7256759 ms`
- Pico最初の通常STOP: `7256775 ms`
- Pico実DriveからSTOP: 約`866 ms`（1000 ms上限以内）
- watchdog冗長STOP: 最終Driveから`500 ms`、UART書込み成功
- 自動再ARM・Drive再送: 0件
- 物理確認: 両輪停止、異音・支持ずれなし、5秒以上再回転なし

証拠:

- `artifacts/roomba_trial_sessions/27121-1793349573-16176/latency.jsonl`

## C1完了判定

**C1 PASS 3/3**。3回の有効な有人両輪浮上試行で、同一generationの連続Drive、
20 mm/s上限、host固定期限STOP、Pico UART STOP、500 ms watchdog、物理停止、
再発進なしを確認した。C2の事前条件のうちC1反復条件は達成した。C2実施は別の
GO/NO-GOと試行個別承認を必要とし、自動的には開始しない。
