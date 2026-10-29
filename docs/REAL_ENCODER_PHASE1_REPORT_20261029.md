# Roomba 680実エンコーダ Phase 1

日付: 2026-10-29
対象branch: `fix-arm-ack-session-recovery`

## 目的と範囲

Roomba OI Query Listでpacket 43（左）と44（右）を取得し、既存の単一TCP/UART所有者を
通じてMacへ返す静止診断経路を追加した。ROS topic、Navigation、TF、ArUco融合、
周期取得はPhase 1の対象外であり変更していない。

## データ経路

1. `roomba_encoder_probe.py`が既存`RoombaTcpTransport`へ接続する。
2. 接続時STOP、STOP ACK `0x07`、左右0、OI Safe mode 2を確認する。
3. 固定TCP診断`D7 07`をPicoへ送る。
4. PicoはOI STOPを最初にUARTへ完全書込みし、ARM状態を失効させる。
5. Picoの同じUART所有者が`149 02 43 44`を送信し、4 byteを固定長で読む。
6. Picoは`D7 87 status left_u16 right_u16`の7 byte固定応答を返す。
7. hostはstatus `0x03`だけを成功とし、左右をunsigned big-endianでCSVへ保存する。

エンコーダ応答type `0x87`はDrive ACK `0x86`と異なる。部分受信、別type、status不完全を
成功として扱わない。取得要求そのものがSTOP-firstであるため、Phase 1 probeは走行監視
には使用せず、静止試験専用とする。

## 安全上の性質

- probeはARMもDriveも送らない。
- Picoはencoder queryより前にSTOPを送る。STOP UART部分書込み時はSTOP latchへ移行し、
  Query Listを送らない。
- query成功時も既存ARMを失効させ、自動再ARMしない。
- `ALLOW_NONZERO_DRIVE=False`、400 ms独立停止、500 ms watchdog、BRC無効を変更しない。
- センサー取得は既存Pico UART所有者内で直列化され、別processはPicoへ直接接続しない。

## 模擬検証

- Pico Query List byte列、左右unsigned 16 bit、STOP-first: PASS。
- UART 3/4 byte部分応答をinvalidとして返す: PASS。
- STOP UART部分書込み時にQueryを行わずlatch: PASS。
- TCP response断片化: PASS。
- TCP 3/7 byte切断、status不完全: PASS。
- probe正常CSV、取得例外のinvalid CSV: PASS。
- 既存decoder、wraparound、odometry: PASS。
- 全回帰: **386/386 PASS**。

## 実機静止試験

個別承認後、`ALLOW_NONZERO_DRIVE=False`、BRC無効、CLEAN手動起床、両輪停止の
条件でSTOP-first probeを10回実行した。

- packet 43（左）: 10/10で`247`。
- packet 44（右）: 10/10で`3948`。
- 静止時差分: 左右とも0 tick。
- valid/connected: 10/10。
- timestamp: host wall/monotonicを保存し、monotonicは全行で増加。
- CSV: `artifacts/real_encoder_phase1/20261029_static.csv`。
- CSV SHA256:
  `f65bffb922b6172ff4474741b3eb3a2f3908ffbea64d28eba4059bbe63781f7c`。
- 終了診断: STOP ACK `0x07`、左右要求0、OI Safe mode 2。
- ARM/Drive: 0件。
- 人間確認: 両輪停止、異音なし、再回転なし。
- 終了: Pico program停止、USB切離し、LED消灯。

左右の絶対countが異なることは累積16 bit値として正常であり、起動時ゼロを仮定しない。

## 判定と未確認

Phase 1は**実機PASS**。実Roombaの左右値を既存UART単一所有者から取得し、MacのCSVへ
保存した。UART timeout/部分応答の実機発生はなく、失敗系はmockのみで確認した。

Phase 2の`/roomba/encoder_counts` publish、`/roomba/odom`接続、通信断後の再基準化、
実移動時の左右符号・距離・角度、ArUco比較は未実装・未確認である。Navigation入力、
TF、自動制御には接続していない。
