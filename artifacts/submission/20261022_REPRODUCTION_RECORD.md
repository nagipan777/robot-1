# PAI最終課題 障害・修正・再現記録

記録日: 2026-10-22  
対象: YOLO / ROS2 / Pico W / Roomba 680統合  
Git基準: `9bd2c29`（未コミット作業を含むため、この値だけでは作業ツリー全体を再現できない）

## 1. この記録の読み方

本書は、変更箇所だけでなく、失敗現象、確認できた直接原因、未確定の根本原因、修正・回避策、
再現結果を対応付ける。推測をPASS証拠として扱わない。

- **確認済み**: ログ、コード、モックテスト、または目視で直接確認した事項
- **有力仮説**: コードと現象が整合するが、発生時の実測が不足する事項
- **未解決**: 原因を一意に特定できていない事項
- STOP限定診断のPASSは、非ゼロDrive成功の証拠ではない
- 合成検出dry-runのPASSは、実カメラYOLO成功や実機走行成功の証拠ではない

## 2. 提出候補の固定構成

- MacBook → Wi-Fi/TCP 9000 → モバイルバッテリー給電Pico W → UART1 GP4/GP5 → Roomba 680
- BRC（Roomba pin 5 / Pico GP2）は物理的に未接続
- RoombaはCLEANボタンで手動起床
- OIはSafe modeのみ。Full modeは禁止
- 通常時は`ALLOW_NONZERO_DRIVE=False`、`drive_enabled=false`、`control_enabled=false`
- Pico watchdogは500 ms

## 3. 障害別の原因・修正・再現結果

### 3.1 BRC接続時のWi-Fi不安定

**失敗現象**

- モバイルバッテリー給電時、初回pingまで数分かかる、または応答が途切れた。
- Pico単体、レベルシフタ接続、UART/BRC接続の条件を分けてpingを採取した。

**原因判定**

- BRCを物理的に外した構成で通信が安定し、手動起床後のSTOP診断が成功したことは確認済み。
- BRC配線が不安定性と相関したことは確認したが、電気的な根本原因は未確定。
- モバイルバッテリー自体を単独の原因と断定できる証拠はない。

**回避策**

- 提出構成ではBRCを接続せず、CLEANで手動起床する。
- BRC自動起床の調査は提出後へ延期する。

**再現証拠**

- `artifacts/pico_ping/20261022_10min_ping_validation.md`
- `artifacts/pico_ping/pico_ping_full-uart-no-brc-confirm_20261022T110416+0900.json`
- 再計測: `python pico_ping_monitor.py 192.168.0.29 --label <条件名> --duration 600`
- BRC再接続による再現試験は現在の提出構成を崩すため実施しない。

### 3.2 Roombaスリープ時のSTOP診断 `status=0x03, 32767/32767`

**失敗現象**

```text
STOP was not confirmed: status=0x03, right=32767, left=32767,
raw=d781037fff7fff
```

**確認できた原因**

- PicoはSTOPフレームを検証し、UARTへ書いたが、眠っているRoombaから要求車輪速度の有効応答を
  得られなかった。`32767`は有効な停止速度ではなく、診断不能を示す代替値。
- CLEANで手動起床した直後、同じBRCなし構成でACK `0x07`、左右`0/0`、OI mode `2`を取得した。

**再現方法（STOP限定で安全）**

1. BRCを未接続にし、Roombaをスリープ状態にする。
2. `./simulation/sim.sh roomba-tcp-diagnostic 192.168.0.29 9000`を実行する。
3. 診断不能またはsentinelを確認する。
4. CLEANで起床し、同じコマンドを再実行する。
5. ACK `0x07`、right/left `0/0`、OI mode `2`をPASS条件とする。

**修正方針**

- sentinelを停止確認成功として扱わず、走行ゲートを開かない現行処理を維持する。
- 起床は提出時には手動操作とする。

### 3.3 Drive実行後のACK欠落 `0/13 bytes`

**失敗現象**

- 左右20 mm/s、最大1秒の床面直進と物理停止は目視確認した。
- その後、ホストは`Drive ACK timed out: received 0/13 bytes`で例外終了した。

**有力仮説と修正**

- Picoソケットをnon-blocking化した後、従来の`send_all()`が一時的な
  `EAGAIN/EWOULDBLOCK`を再試行せずACKを失う経路があった。
- `pico/pico_relay.py`で一時的send失敗を最大100 msだけ再試行するよう最小修正した。
- `simulation/roomba_bridge_tcp.py`で診断例外を既存のSTOP・ゲート取消・切断・再接続へ送るようにした。
- TCPフレーム形式、CRC、世代、sequence、500 ms watchdogは変更していない。

**検証**

- EAGAIN後の回復、永続失敗、分割・連結受信をモックで検証した。
- 発生時Picoログがないため、この仮説だけで元の実機障害の根本原因確定とはしない。
- 詳細: `artifacts/floor_drive_tests/20261021_drive_ack_timeout_report.md`

### 3.4 Drive ACK `status=0x01`（UART全5バイト未完了）

**失敗現象**

```text
Drive rejected: status=0x01, generation=1792638939, sequence=6
```

**確認できた直接原因**

- `0x01`はDriveフレーム検証成功bitだけが立ち、UART全書込み成功bit `0x02`がない状態。
- CRC、generation、sequenceの検証後、`uart.write()`がDrive Direct 5バイトを完全に受理しなかった。
- この現象はBRCなし・事前STOP/Safe診断PASS後にも再現した。

**未確定の根本原因**

- UARTバッファ枯渇、MicroPython実行タイミング、電源・配線、配備版差異のどれかは未分離。
- 実配備`main.py`とリポジトリ`pico/pico_relay.py`の認証情報除外後の完全照合も未完了。

**安全側の修正・動作**

- `uart.write()`の戻り値`5 / 4 / 0 / None`をモック化した。
- `4 / 0 / None`では成功ACKを返さず、sequenceを進めず、STOPを試行して`LATCHED_STOP`へ移る。
- OIはフレーム区切りを持たないため、残りバイトの追送やDrive全体の自動再送はしない。
- 異常後の再接続でSTOP ACK `0x07`、左右`0/0`、Safe `2`へ復帰した。

**再現証拠**

- `artifacts/floor_drive_tests/20261022_arm_drive_ack_timeout.md`
- 実機での再現コマンドは現在NO-GOのため、本書からは実行しない。
- モック再現: `python -m unittest tests.test_pico_relay tests.test_pico_relay_protocol -v`

### 3.5 ARM/STOP診断の応答timeout

**失敗現象**

- ARM response `0/7 bytes`、STOP diagnostic `0/7 bytes`、またはREADY `0/6 bytes`が散発した。
- 再接続後に同じ診断が成功する例もあった。

**原因判定**

- ARMはUARTを使わないため、ARM timeoutはRoomba UART部分書込みだけでは説明できない。
- TCP応答、Pico ACK送信、実行中配備版、Pico処理停止のどれかは未確定。

**現行の安全動作**

- timeout時は走行許可を取り消し、STOPを試行し、切断・再接続する。
- 再接続だけでは古いTwistを再利用せず、新規ARMが必要。
- 初回診断失敗を後続成功で隠さず、その試行全体をFAIL/HOLDと記録する。

### 3.6 合成検出dry-runの最初の失敗と再現成功

**最初の失敗**

- 合成HTTP検出を速く連続送信すると、検出metadataが次フレームで上書きされ、
  `missing_detection_metadata`で目標候補が拒否された。
- これは実YOLOの失敗ではなく、検証入力の送信間隔による試験条件の問題。

**再現できた修正条件**

- 既到達扱いの同一目標を避け、十分離れた新規目標を1 Hzで送信した。
- 画像座標`(200,700)`、confidence `0.95`からmap座標`(1.1076,0.7028)`を生成。
- `/nav/cmd_vel_unstamped`非ゼロ8件、`/roomba/cmd_vel_dry_run`非ゼロ8件、停止2件を確認。
- 非ゼロは旋回`linear.x=0.0, angular.z=0.8`で、検出途絶後はゼロになった。
- `/roomba/cmd_vel_unstamped`は存在せず、TCP 9000へ接続していない。

**証拠**

- `artifacts/e2e_real_camera_pico_dry_run/result-A.json`
- `REAL_ROOMBA_AUTONOMY_CRITICAL_PATH.md`
- これは合成検出によるROS経路PASS。実カメラYOLOは別途未実施。

## 4. 変更ファイルと変更理由

| ファイル | 変更理由 |
|---|---|
| `pico/pico_relay.py` | ACK送信時の一時的EAGAIN再試行、UART不完全書込みのfail-closed維持 |
| `simulation/roomba_bridge_tcp.py` | ACK/診断例外をSTOP・ゲート取消・再接続へ統一 |
| `tests/test_pico_relay.py` | UART write戻り値5/4/0/None、ACK送信失敗等の再現 |
| `tests/test_pico_relay_protocol.py` | CRC・世代・sequence・分割/連結フレーム回帰 |
| `pico_ping_monitor.py` / `tests/test_pico_ping_monitor.py` | 給電・配線条件別の時刻付きping収集と集計 |
| `simulation/vision_turn_controller.py`および設定/テスト | dry-run経路と安全な目標管理の既存変更。実機出力とは分離 |

本書作成時点の主要ファイルSHA-256:

```text
pico/pico_relay.py                              c84d0f93a5f776520a24fa2c8843115cc4dd8e694f5eab953ca49a4d5e35dd72
simulation/roomba_bridge_tcp.py                 f9885c14ccbdaf48c9a96935eca157915eee55cc6846da3bedf8b492e73d090b
simulation/roomba_tcp_transport.py              dfff8a8e82472336dac1fe89a5457388d9114fe1b1c17d8899a99bd28fd8230d
simulation/vision_turn_controller.py            70e050d30837a21ad6ba10282fe80f194052bef377ae1426d46fe614713506a1
simulation/vision_controller.real_dry_run.yaml   fe65c9729132e28bb0f2c7e2a269693937e92da67168095556a55118605e9850
pico_ping_monitor.py                            e62108f21b9ea7ed4ccd5ed25da6cd9163695fa388820268a45618faa50141c4
```

## 5. 現時点の再現可能な安全確認

### STOP限定診断

```bash
cd /Users/imaichika/robot
./simulation/sim.sh roomba-tcp-diagnostic 192.168.0.29 9000
```

PASS条件:

```text
relay READY
STOP confirmed ... ack=0x07 ... right=0 ... left=0
Roomba OI mode: 2 (Safe)
STOP-only mode: drive_enabled=false
```

### 合成検出dry-run

```bash
cd /Users/imaichika/robot
./simulation/sim.sh real-camera-dry-run-up
```

起動後、`control_enabled=false`、`calibration_ready=false`、`dry_run_enabled=true`、
`/roomba/cmd_vel_unstamped` Publisher 0を確認する。実機IPを渡さずTCPブリッジを起動しない。

## 6. 現在の判定と次段階の証拠

| 対象 | 判定 | 次へ進むための不足証拠 |
|---|---|---|
| STOP限定診断 | GO | 毎回ACK 0x07、左右0、Safe=2を確認 |
| 合成検出ROS dry-run | PASS | 実カメラYOLOは別試験 |
| 実カメラYOLO dry-run | 未実施 | bbox、confidence、map目標、dry-run Twist、最終ゼロを同一試行で保存 |
| 非ゼロDrive信頼性 | FAIL/HOLD | 配備版照合と、全sequenceでDrive ACK 0x03。0x01/timeout/再接続0件 |
| 車輪浮上試験 | HOLD | 上記Drive信頼性を解決後、20 mm/s・1秒以下で再確認 |
| 有人限定床面走行 | NO-GO | 車輪浮上PASS、物理封じ込め、単一Publisher、終了後STOP診断 |
| YOLO連動有人自律 | NO-GO | 実カメラdry-run、ArUco姿勢鮮度、車輪浮上・床面の段階試験を全てPASS |
| 無人自律走行 | NO-GO | Pico電源断/UART断時にSTOP不能。独立停止手段が未整備 |

## 7. 記録時の安全状態

- この記録作成では実機接続、TCP 9000接続、UART送信、ロック解除、非ゼロ走行を実施していない。
- 期待する通常状態は`ALLOW_NONZERO_DRIVE=False`、`drive_enabled=false`、
  `control_enabled=false`、`calibration_ready=false`。
- 次回は実カメラYOLO dry-runを先に完了し、UART部分書込みを解決するまで実機非ゼロDriveへ進まない。

