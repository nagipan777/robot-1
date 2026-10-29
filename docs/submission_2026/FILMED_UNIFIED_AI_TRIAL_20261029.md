# 2026-10-29 単一ブリッジAI連動・撮影試験

## 判定

**有人実機単発試験PASS（限定条件）**。固定環境の実カメラでmaskの`litter` bboxと
ArUco ID 0を維持し、新鮮なNavigation候補を確認してからgeneration
`1793253223`を1回だけARMした。Roombaは短時間動作し、その後、人間が両輪停止、
異音なし、再発進なしを確認した。試験映像はユーザー端末へ保存されており、
リポジトリ内ファイルの存在は未確認である。

## 構成と事前条件

- bridgeは`roomba-trial-session`による単一プロセス。
- 起動時STOP ACK `0x07`、左右要求速度0、OI Safe mode 2。
- model v2、confidence 0.50、imgsz 960、USB camera 0、800x600構成。
- BRC未接続、CLEAN手動起床。
- Picoは試行時のみ`ALLOW_NONZERO_DRIVE=True`。400 ms独立停止、500 ms
  watchdog、20 mm/s上限を維持。
- physical command topicはpublisher 1 / subscriber 1。ARM subscriber 1。
- ping 10/10、loss 0%、Mac-Colima clock skew 0.289 s。
- 個別の有人試行承認後にのみARMした。

## 保存traceの実測

証拠:
`artifacts/roomba_trial_sessions/24676-1793252443-14845/latency.jsonl`

- ARM ACK: accepted、generation `1793253223`。
- Drive ACK: sequence 1–4の4/4、status `0x03`。
- wheel request: right 20 mm/s、left 12–13 mm/s。上限超過なし。
- 最初のDrive送信: monotonic `52568.719091102`。
- deadline STOP開始: monotonic `52569.022735186`。
- 最初のDriveからSTOP開始: **303.644 ms**。
- セッションdeadlineからSTOP開始の超過: **1.126 ms**。
- STOP TCP send: **0.185 ms**。
- deadline後のDrive送信: **0件**。
- deadline STOP後の非ゼロ候補は`transport_disconnected`または
  `arm_required`で拒否され、自動再ARM・再Driveはなかった。

deadline検出はlive Safe照会の応答待ち中に成立し、deadline STOPが同期I/Oを中断して
接続を安全側へ閉じた。これにより、P3-2で意図したSTOP優先経路を実通信で確認した。
Pico UART STOP完了時刻は今回保存したホストtraceだけからは確定せず、物理停止は
人間の目視証拠として区別する。

## 終了確認

- 出力アダプターを停止。
- `ALLOW_NONZERO_DRIVE=False`へ復帰し、4安全定数を再確認。
- 終了診断でSTOP ACK `0x07`、左右0、OI Safe mode 2。
- Pico側で新規TCP接続、READY 6 byte、`normal STOP forwarded`を確認。
- Picoプログラム停止後にUSBを外し、LED消灯。
- USBカメラを終了・切離し。
- 人間が両輪停止、異音なし、再発進なしを確認。

## 残存する運用課題

1. 非TTYで起動したphysical output adapterが試験終了後も残り、約10 Hzで候補を
   publishし続けた。ARM失効により全件拒否され実機は再発進しなかったが、次回は
   session cleanupがadapter PIDも確実に終了・検査する必要がある。
2. Pico再起動後、旧TCP socketへのゼロSTOP書込みが一時的に成功扱いとなり、Pico側へ
   到達しない状態があった。bridge再起動後のSTOP専用診断はPASSした。再起動を伴う
   場合は同一socketの自然回復に依存せず、明示的にsessionを終了して再診断する。
3. 撮影ファイルはユーザー端末側にあり、提出素材へコピーしたパスとhashは未記録。
