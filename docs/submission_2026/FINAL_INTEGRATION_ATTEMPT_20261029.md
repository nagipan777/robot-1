# 2026-10-29 最終AI統合試行記録

## 判定

**試行FAIL（安全側停止）**。ARM ACKは成立したが、ARM後に新鮮なNavigation候補が
bridgeへ到着せず、Driveは0件、物理移動はなかった。再ARM・再Driveは行っていない。

## 事前ゲート

- Pico配備版はSSID/PASSWORD正規化後に`pico/main.py`と完全一致。
  SHA256: `230b203612481c5906e269b5c463293b079c8b15ba2aebdbb5df6a354f788931`。
- STOP ACK `0x07`、要求左右0、OI Safe mode 2。
- Pico ping 10/10、loss 0%、RTT 4.423/15.307/53.521 ms min/mean/max。
- Mac–Colima clock skew 0.115 s。
- BRC未接続、UART配線・レベル変換・給電に異常なし。
- model v2、confidence 0.50、imgsz 960、800×600 camera 0。
- 本日のmask検出は正面60 cmで安定し、70 cmでは0.52–0.55で不安定だった。
- cmd topic publisher 1 / subscriber 1、ARM publisher 0 / subscriber 1。
- 20 mm/s上限、400 ms host/Pico期限、500 ms watchdogを維持。

## 試行時系列

- controller再起動後、ARM前に`/nav/cmd_vel_unstamped`の
  `linear.x=0.08 m/s`候補を確認した。
- Navigation/adapterは`1793238542.557`に通常STOP/timeout STOPとなった。
- ARM generation `1793238564`を1回だけpublish。
- bridgeは`1793238565.818`にARM acceptedを記録した。
- 最後のSTOPからARM受理までは約23.26秒で、ARM後のDrive ACKは0件だった。
- したがってARM/TCP障害ではなく、ARM時点で新鮮なNavigation非ゼロ候補がなかった
  ことが直接の不成立条件である。対象管理内部の詳細原因はこの試行だけでは断定しない。

## 終了状態

- bridgeとadapterを終了し、残存PIDを個別に除去。
- `ALLOW_NONZERO_DRIVE=False`へ復帰し、4安全定数を再確認。
- 終了STOP ACK `0x07`、要求左右0、Safe 2。
- camera/ROS/container/networkを終了。
- Pico TCP/UARTを停止してUSBを抜き、LED消灯。
- 人間が両輪停止、異音なし、再発進なしを確認。

## 次回の最小改善

追加走行前に、ARM操作を人間応答時間へ依存させず、承認済み試行の中で「新鮮な
Navigation非ゼロ候補を観測→直後に単一ARM」を同一の非走行確認スクリプトまたは
明示的な手順で同期させる。自動再ARMは追加しない。まずPico lock False・物理bridge
なしで、その同期手順が1回だけARM候補を生成し、古い候補では何もしないことを
dry-run検証する。

この同期補助は同日実装され、mock ROS 2 E2Eと全回帰380/380をPASSした。さらに
実カメラ、実ArUco姿勢、実mask検出から生成された新鮮なNavigation候補で、模擬ARM
subscriberへgenerationを1回だけ送る非走行E2EをPASSした。古いArUco姿勢ではARM
0件のままtimeoutし、controller/姿勢状態の明示的な初期化後だけ候補が復帰した。
mask除去後は非ゼロ候補0件、再ARM 0件、物理Roomba topicは全期間不存在だった。
実Pico/Roombaとの同期確認は別ゲートとして残す。詳細は
`docs/submission_2026/ATTENDED_NAV_ARM_SYNC_REPORT_20261029.md`を参照する。

