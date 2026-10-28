# 10月29日 AI連動実機試験・期限解析

> 追補: ホスト側ブロッキングI/O修正とdry-run結果は
> `P3_2_DEADLINE_IO_SEPARATION_REPORT_20261029.md` に記録した。本書の実機
> 結果は変更せず、dry-run PASSを実機PASSとして扱わない。

## 結論

成果は分離して扱う。

- **AI連動実機経路: PASS** — YOLO、ArUco、Navigation、出力アダプター、ARM、Pico、Roombaまで到達し、左右20 mm/sのDrive ACK `0x03`を5件確認した。人間が短時間の実動、物理停止、異音なし、再発進なしを確認した。
- **ホスト400 ms期限: FAIL** — ホストのSTOP TCP送信開始はセッション開始から441.812 msで、400 msを41.812 ms超過した。期限検知自体は41.348 ms遅れた。
- **終了安全状態: PASS** — STOP ACK `0x07`、左右要求速度0、OI Safe mode 2、`ALLOW_NONZERO_DRIVE=False`、ROS実機経路停止、Pico無通電を確認した。

追加走行は実施していない。

## ホスト側タイムライン

ホストの`time.monotonic()`だけで相対時間を算出した。Picoの`time.ticks_ms()`とは直接比較していない。

| 事象 | host monotonic [s] | セッション開始から [ms] |
|---|---:|---:|
| bounded session開始 | 24499.666371 | 0.000 |
| Drive ACK seq=1 | 24499.897674 | 231.303 |
| Drive ACK seq=2 | 24499.914357 | 247.986 |
| Drive ACK seq=3 | 24499.926444 | 260.073 |
| Drive ACK seq=4 | 24499.978748 | 312.377 |
| Drive ACK seq=5 | 24500.059898 | 393.527 |
| 固定期限 | 24500.066371 | 400.000 |
| 期限検知 | 24500.107719 | 441.348 |
| STOP TCP送信開始 | 24500.108183 | 441.812 |
| STOP TCP送信呼出し復帰 | 24500.108232 | 441.861 |

Navigation候補はARM前から到着していたが、ブリッジは明示ARMまで全非ゼロ指令を拒否した。ARM generation `1793220505`の受理後だけDriveが成立し、期限時にARMを失効した。その後の非ゼロ候補はすべて`explicit ARM generation is required`で拒否された。

## 41 ms遅延の内訳

| 区間 | 実測 | 判定 |
|---|---:|---|
| 期限から期限検知 | 41.348 ms | 主要因。20 Hz（50 ms周期）timerの位相遅れと整合する |
| 期限検知からSTOP送信開始 | 0.464 ms | 小さい |
| STOP TCP送信呼出し | 0.049 ms | 小さい |
| Safe照会 | 個別値なし | この試行ではtrace未保存のため推定しない |
| Drive ACK待ち | 個別値なし | この試行ではtrace未保存のため推定しない |
| ROS callback単体 | 個別値なし | timer/executor遅延から分離不能 |

最後のDrive ACKは期限の6.473 ms前に完了した。そこから期限検知までは47.821 msで、次の20 Hz tickを待った挙動と一致する。TCP STOP送信そのものは原因ではない。

## Pico独立400 ms期限

この試行のPico consoleから、次のどちらかを示す行を保存していない。

- `session deadline detected ...`
- `session deadline UART STOP complete ...`
- `normal STOP forwarded ...`

したがって、Pico独立期限と通常STOPの先後は**UNKNOWN**とする。ホストとPicoのmonotonic時計を壁時計のように比較して結論を作らない。コード上はPico期限が最初の有効DriveのUART転送後に開始し、5 ms周期で監視されるが、今回の直接証拠の代用にはしない。

## 非走行再現

`simulation/deadline_dry_run.py`を実機未接続・ARM送信なしで実行した。

- 検知遅れ: 35.049 ms
- 期限後socket Drive送信: 0件
- 結果: PASS

既存の2026-10-27 artifactでも38.569 msを記録しており、今回の41.348 msと同じオーダーで再現した。

## 最小修正

`simulation/roomba_bridge_tcp.py`に、既存20 Hz I/O tickとは独立した100 Hz期限監視timerを追加した。

- 400 ms、500 ms、20 mm/s、ARM、STOP latchは変更しない。
- 通常の20 Hz Drive/Safe/TCP処理は変更しない。
- 新timerは有効なbounded sessionの期限だけを監視し、idle TCP通信を増やさない。
- 期限後DriveをTCP送信直前にも拒否する既存防御は維持する。

この修正は通常のsingle-threaded ROS executor上で期限検知の位相上限を約50 msから約10 msへ縮小する。ただし、同期Safe照会またはACK待ちでexecutor自体がブロックした場合の独立性までは保証しない。そこはPico独立400 ms期限が最終防御であり、次回は低負荷の非走行ROSモックでtraceを保存して確認する。

## 検証

- 非走行deadline dry-run: PASS
- 期限後Drive送信0件: PASS
- ROS2 SingleThreadedExecutor非走行30試行: PASS
  - 旧20 Hz期限監視（15試行）: 最大検知遅延29.950 ms
  - 修正100 Hz期限監視（15試行）: 最大検知遅延9.462 ms
  - callback模擬負荷: 0、3、8 ms
  - 期限後の模擬Drive送信: 0件
  - ネットワーク、Pico、ARM、物理出力topic: 不使用
- ARM/ACK、STOP latch、Pico独立期限を含む対象回帰: 104/104 PASS
- リポジトリ正規コマンドによる全回帰: 370/370 PASS
- 実機での修正版期限遵守: **未確認**（今回の指示により再走行禁止）

## 提出用記述

「10月29日、実カメラのYOLO検出とArUco姿勢をNavigation指令へ変換し、ARM安全ゲートを通してRoomba 680を左右20 mm/sで初めてAI連動駆動した。Drive ACKを5件取得し、人間が停止と再発進なしを確認した。一方、ホスト側400 ms期限のSTOP送信開始は441.812 msであり、41.812 msの時間制御課題を残した。期限後のDriveは拒否され、最終状態はSTOP ACK 0x07、左右0、OI Safe mode 2、走行ロック有効、Pico無通電である。」

## 100 Hz修正版・有人実機再検証

同日、模擬試験PASSを実機PASSとみなさず、左右20 mm/sの有人単発試験を1回だけ実施した。追加ARM・再Driveは行っていない。

### ホスト側

- bounded session開始: `27576.178542`
- 固定期限: `27576.578542`
- 期限検知: `27576.759791`
- 検知遅延: **181.249 ms**
- STOP TCP送信開始: `27576.761678`（開始から**583.136 ms**）
- STOP TCP送信処理: 0.483 ms
- Drive ACK: 6件
- 期限後Drive ACK: 0件

100 Hz timerの非走行30試行では最大9.462 msだったが、実通信ではsingle-threaded executor上の同期Safe照会またはDrive ACK待ちがtimer callbackをブロックした可能性が高い。STOP TCP送信そのものは0.483 msであり、主要因ではない。個別traceを保存していないため、Safe照会とACK待ちの内訳は推定で埋めない。

### Pico側

Picoの`time.ticks_ms()`内だけで相対時間を算出した。

- 最初のDrive UART転送: `654425 ms`
- 独立期限検知: `654829 ms`
- 相対時間: **404 ms**
- UART STOP完了: `654830 ms`、所要0 ms、`uart_written=True`
- 最初の通常STOP: `654936 ms`
- 期限後Drive転送: 0件

Pico独立期限が通常STOPより先に発火し、物理停止を担保した。ただし厳密な400 ms以内という判定では4 ms超過のためFAILである。5 ms receive loop周期と整合するが、閾値を緩和せず改善対象とする。

### 更新後のP3判定

- 100 Hzホスト期限: **FAIL**
- Pico厳密400 ms期限: **FAIL**
- Pico独立STOP動作: **PASS**
- 期限後Drive 0件: **PASS**
- 物理停止、異音なし、再発進なし: **PASS**
- P3期限ゲート: **NO-GO（修正継続可能）**

次の最小作業は、ホストの同期Safe照会・Drive ACK待ち時間をtraceで分離し、deadline STOPをブロッキングI/Oから独立させる設計を非走行モックで検証することである。Pico側は400 ms判定を変更せず、5 ms loopの期限確認順序または待機方法を見直す。実機再試行は両方の非走行回帰がPASSした後に個別承認を得て行う。
