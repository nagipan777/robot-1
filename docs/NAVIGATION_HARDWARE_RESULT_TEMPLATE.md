# Navigation実機単体試験 結果記録テンプレート

## セッション情報

- 日時:
- branch / commit:
- trial ID:
- 実施者:
- Pico配備コード照合:
- Pico IP / RSSI:
- Roomba電池:
- BRC未接続: YES / NO
- 両輪浮上・支持安定: YES / NO
- 独立停止手段: YES / NO
- 人間の個別承認文:

## 安全設定

| 項目 | 期待 | 実測 | PASS/FAIL |
|---|---|---|---|
| `ALLOW_NONZERO_DRIVE` 通常状態 | False | | |
| BRC wake | False | | |
| Pico session limit | 400 ms | | |
| watchdog | 500 ms | | |
| bridge max speed | 20 mm/s | | |
| bridge session limit | 0.4 s | | |
| STOP ACK | 0x07 | | |
| requested right/left | 0/0 | | |
| OI mode | 2 Safe | | |
| physical bridge count | 1 | | |
| ARM subscriber count | 1 | | |
| command subscriber count | 1 | | |
| latency trace path | 非空 | | |

## 試行入力と期待値

- 試行: 直進 / 左旋回 / 右旋回 / STOP
- Navigation mode: turn_only / approach
- robot pose `(x,y,yaw)`:
- target `(x,y)`:
- distance:
- angle error:
- 期待`linear.x`:
- 期待`angular.z`:
- 期待left/right mm/s:
- 期待物理方向:

## 通信・停止ログ

- ARM generation:
- ARM ACK時刻/status:
- Drive sequence一覧:
- Drive ACK一覧（status/right/left）:
- 最初のDrive時刻:
- host deadline:
- host STOP begin / return:
- Pico STOP reason:
- Pico UART STOP start / complete:
- STOP ACK時刻/status:
- 期限後Drive件数:
- TCP切断/再接続:

## 物理観測

- 左輪方向:
- 右輪方向:
- 機体旋回方向:
- 物理停止確認時刻:
- 異音:
- 支持台ずれ:
- STOP後5秒の再発進: なし / あり

## ArUco前後比較（実施時のみ）

| 項目 | 開始 | 終了 | 差 |
|---|---:|---:|---:|
| timestamp | | | |
| x [m] | | | |
| y [m] | | | |
| yaw [rad] | | | 正規化値 |

- 目標角度:
- 位置変化`hypot(dx,dy)`:
- 実測角度差:
- 欠損数:
- 外れ値数と除外根拠:
- 較正誤差/分解能を上回る: YES / NO
- 目視方向と一致: YES / NO

## 合否

- 前提条件: PASS / FAIL
- Twist/輪符号: PASS / FAIL
- Drive ACK: PASS / FAIL
- 物理方向: PASS / FAIL
- STOP/UART/物理停止: PASS / FAIL
- 再発進なし: PASS / FAIL
- ArUco測定: PASS / FAIL / INVALID / NOT RUN
- 総合: PASS / FAIL / ABORTED
- 中止理由:
- 未確認事項:
- 次段階へ移行: YES / NO

## 後片付け

- ゼロTwist/STOP確認:
- bridge終了:
- `ALLOW_NONZERO_DRIVE=False`復帰:
- 最終STOP ACK 0x07 / 0/0 / Safe 2:
- Pico USB切断・LED消灯:
- 両輪停止・異音なし・再発進なし:
- ROS/trace/Picoログ保存先:
