# Roomba 680 連続ARM・直進試験 設計メモ

更新日: 2026-10-30
状態: 非走行設計・モック検証。Pico配備および実機走行は未実施。

## 1. 現状

PicoのARMはboolではない。`MotionLatch`が次を同時に満たすDriveだけを受理する。

- stateが`ARMED`
- ARM ACK完了後にcommitされたgenerationと一致
- ARM時のTCP connection IDと一致
- sequenceが最後に受理した値より大きい
- 最初のDriveから固定セッション期限内
- `ALLOW_NONZERO_DRIVE=True`

最初のDrive受理後もARMは維持され、同じgenerationでsequence 1, 2, 3...を受理できる。
各受理Driveは500 ms watchdog用の最終コマンド時刻を更新する。一方、既存400 ms期限は
最初のDrive時刻から非更新であり、Driveを継続しても延長されない。

ホストも`DriveSessionLimiter`で最初の非ゼロ候補から400 msの非更新期限を持つ。
STOP、期限到達、ACK不明、Safe喪失、切断ではARMを失効し、自動再ARMしない。

過去実測では同一generationで複数Driveが成立している。

- 2026-10-28 AI床面試験: Drive ACK sequence 1--7、最初のDriveから通常STOP約289 ms。
- 2026-10-29 P3-2浮上試験: Pico Drive sequence 1--7、最初のDriveから通常STOP224 ms。

したがって課題はARMの複数Drive対応ではなく、ホスト/Pico双方の固定400 ms期限を、
既存モードから分離して明示的に拡張することである。

## 2. 二つのモード

| 項目 | 既存実証モード | 有人連続直進モード |
|---|---:|---:|
| Pico opt-in | 不要 | `CONTINUOUS_DRIVE_TEST_ENABLED=True` |
| Pico固定期限 | 400 ms | 1000--5000 ms、試行ごとに明示 |
| host fixed deadline | 400 ms | 1.0--5.0 s、Picoと一致必須 |
| watchdog | 500 ms | 500 ms（変更なし） |
| speed ceiling | 20 mm/s | 20 mm/s |
| ARM | 明示generation | 明示generation、同一session内で維持 |
| STOP後 | ARM失効 | ARM失効 |
| 自動再ARM | なし | なし |
| bridge | 単一 | 同じく単一 |
| 専用確認文字列 | 既存値 | `CONTINUOUS_ATTENDED_FLOOR` |

通常モードの`DRIVE_SESSION_TIMEOUT_MS=400`と`DriveSessionLimiter`既定上限1秒は変更して
いない。連続モードを明示した場合だけ、別上限5秒を渡す。

## 3. 状態遷移

```text
STOPPED
  -- explicit ARM(generation, connection_id) --> ARM_PENDING
  -- complete ARM ACK send/commit -----------> ARMED, sequence=0
ARMED
  -- valid Drive sequence=n -----------------> ARMED, sequence=n
  -- next valid Drive <500 ms ---------------> ARMED, sequence=n+1
  -- no valid Drive for 500 ms --------------> LATCHED_STOP
  -- fixed experiment deadline --------------> LATCHED_STOP
  -- STOP / Safe loss / TCP loss / UART error -> STOPPED or LATCHED_STOP
STOPPED/LATCHED_STOP
  -- old generation or delayed Drive --------> reject
  -- automatic ARM --------------------------> prohibited
```

ホストはACK確定後だけsequenceを進める。ACK timeoutまたは部分受信では同じDriveを
再送せず、STOPと接続再同期へ移る。

## 4. 距離段階

速度は20 mm/sに固定し、時間だけを一変数として拡張する。

| 段階 | host/Pico固定期限 | 理論距離 | 目的 |
|---|---:|---:|---|
| C1 | 1.0 s / 1000 ms | 20 mm | 400ms超でARM維持・複数ACK確認 |
| C2 | 2.0 s / 2000 ms | 40 mm | watchdog余裕・停止再現性 |
| C3 | 3.0 s / 3000 ms | 60 mm | 第一目標、ArUco変位評価 |
| C4 | 5.0 s / 5000 ms | 100 mm | C3再現後だけ実施 |

理論距離は速度×期限であり、加減速、床面滑り、STOP遅延を含まない。実測距離を
ArUco前後姿勢から別記録し、未統合エンコーダ値を実移動距離として扱わない。

## 5. 故障モードと代替条件

| 故障モード | 根拠/観測 | 試行判断 | 検証可能な代替 |
|---|---|---|---|
| Drive間隔が500 ms以上 | Pico watchdog発火 | 当該試行終了 | 浮上・1秒、送信周期100 msで再測定 |
| host/Pico期限不一致 | 一方だけ先にSTOP、原因が曖昧 | 開始不可 | 両方1000 msに戻す |
| ACK timeout/部分応答 | Drive実行有無が不明 | STOP、再送なし | ロックFalseで遅延注入試験 |
| sequence欠落/重複 | ACK混線または再送 | 試行終了 | mockでsequence 1..Nを再確認 |
| Safe喪失 | OI mode !=2 | 即STOP | 静止診断のみ再実施 |
| ArUco欠損 | 距離を定量評価不能 | 通信試験のみ判定 | 浮上または目視方向試験へ縮小 |
| 斜行 | 左右差、床面、機械差 | 速度を上げない | 20 mm/s・1秒へ戻し前後姿勢記録 |
| STOP後再発進 | latch/古い候補の破綻 | 以後の走行中止 | 非走行のSTOP後Drive拒否へ戻る |

## 6. Navigation接続条件

60 mm直進を少なくとも3回再現し、次を満たした後に限る。

- 各試行でACK sequenceが単調増加し欠落理由を説明できる。
- 最大Drive間隔が500 ms未満で、p95と最大値を記録できる。
- host/Pico固定期限とSTOP時刻を相対時間で照合できる。
- STOP後5秒間再発進なし。
- ArUco変位が較正分解能を上回り、目視前進方向と一致。
- stamped Navigation候補、または同等の入力生成時刻検査が送信直前に成立する。

その後も直進・旋回切替、姿勢再取得、角度補正は別段階とし、自動再ARMは追加しない。

## 7. 非走行検証結果

- 同一generation/connection IDでsequence 1--8を400 ms間隔で受理: PASS。
- 最初のDriveから3秒境界のsequence 9をUART送信前に拒否: PASS。
- 3秒連続モード中も、Drive途絶500 msでSTOP・STOP latch: PASS。
- host/Pico期限不一致、20 mm/s超過、5秒超、専用確認文字列不一致の拒否: PASS。
- 400 ms既存モードと既定1秒上限の非変更: PASS。
- ACK 0--300 ms、timeout、部分応答、TCP再接続、Safe喪失、旧generation拒否: PASS。
- 全回帰: 417/417 PASS。
- Pico実配備、実Wi-Fi、実UART、物理停止、実移動距離: UNVERIFIED。
