# C2連続直進試験 GO/NO-GO判定

日付: 2026-10-30  
対象: 両輪浮上、左右20 mm/s、host/Pico最大2.0秒（理論40 mm相当）

## 結論

- **C1反復条件: PASS（3/3）**
- **C2両輪浮上試験: PASS（3/3）**
- **C3非走行準備: GO**
- **C3両輪浮上試験: 条件付きGO（再設定・非走行診断・試行別承認が必要）**
- **C3床面60 mm評価: 現時点NO-GO**

C2終了後、Picoは安全な標準状態
（`CONTINUOUS_DRIVE_TEST_ENABLED=False`、`ALLOW_NONZERO_DRIVE=False`）へ復帰済みである。
C3床面評価をNO-GOとする理由はC2の不具合ではない。C2は両輪浮上であり、理論40 mmに
対応する通信継続と停止を検証したが、床面変位、斜行、ArUco前後姿勢を測定していない。

## C2実機結果

| 試行 | generation | ACK | 最大Drive間隔 | 最大ACK待ち | host期限遅れ | STOP TCP | 期限後Drive | 物理停止 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | 1793350829 | 28 | 397.895 ms | 192.440 ms | 1.232 ms | 0.586 ms | 0 | PASS |
| 2 | 1793351297 | 32 | 278.418 ms | 166.083 ms | 2.847 ms | 0.107 ms | 0 | PASS |
| 3 | 1793351720 | 26 | 481.737 ms | 424.660 ms | 1.815 ms | 0.208 ms | 0 | PASS |

全試行で同一TCP session・同一ARM generationのsequenceが1から単調増加し、Drive ACK
status `0x03`、左右要求速度`20/20`、期限後Drive 0件、自動再ARM・再Drive 0件を確認した。
人間の目視では両輪停止、異音・支持ずれなし、5秒以上再回転なしを確認した。

試行3の最大Drive間隔481.737 msは500 ms watchdog未満でPASSだが、余裕は18.263 msしかない。
C3ではこの値を通信ジッター監視条件とし、500 ms到達時は想定どおりwatchdog STOPとなる
ことを試験失敗として隠さず記録する。

## C1有効試行の実測

| 試行 | generation | ACK | 最大Drive間隔 | 最大ACK待ち | host期限遅れ | STOP TCP | Pico実Drive→STOP | 物理停止 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | 1793344989 | 13 | 126.809 ms | 125.550 ms | 0.832 ms | 0.634 ms | 最大1秒内 | PASS |
| 2 | 1793349121 | 17 | 164.183 ms | 81.016 ms | 1.775 ms | 0.138 ms | 864 ms | PASS |
| 3 | 1793349637 | 13 | 303.012 ms | 170.735 ms | 0.593 ms | 0.101 ms | 866 ms | PASS |

全試行で次を確認した。

- 同一TCP session・同一ARM generationで複数sequenceを受理
- Drive ACK status `0x03`、左右要求速度`20/20`
- Drive間隔は500 ms watchdog未満
- host期限後Drive 0件
- STOP後の自動再ARM・Drive再送 0件
- Pico UART STOP書込み成功
- 両輪停止、異音・支持ずれなし、5秒以上再回転なし
- 終了後STOP `0x07`、左右0、OI Safe 2、二重ロックFalse

## C1中に見つけて修正した運用問題

試験publisherが固定待ち時間だけでARM完了を推定していたため、pre-ARM STOP/Safe診断と
Pico ARM ACK検証中にも走行時間を消費していた。ブリッジが検証済みgenerationを
`/roomba/arm_accepted_generation`へ通知し、publisherが同一generationを受信するまで
非ゼロをpublishしないよう修正した。通知なしの場合は5秒後にSTOPのみで終了する。

修正後の焦点テストは43/43、全回帰は422/422 PASS。Pico lock Falseの非走行実通信で
ARM拒否、受理通知なし、非ゼロ0件、timeout後STOPのみを確認した。

## C3両輪浮上試験をGOにする最小条件

1. C3を両輪浮上専用の個別試行とし、床面走行・Navigationへ接続しない。
2. Picoの試行限定設定を`CONTINUOUS_DRIVE_SESSION_TIMEOUT_MS=3000`へ変更し、
   `CONTINUOUS_DRIVE_TEST_ENABLED=True`と`ALLOW_NONZERO_DRIVE=True`は当該試行だけ有効化する。
3. host launcherを3.0秒で起動し、Picoとhostの期限一致を起動前に照合する。
4. STOP `0x07`、左右0、Safe 2、単一bridge、孤児publisher 0件を非走行で再確認する。
5. 両輪完全浮上、支持安定、配線干渉なし、独立停止手段を再確認する。
6. C2試行3で観測した最大Drive間隔481.737 msを踏まえ、500 ms watchdog発火を中止条件として
   ログに残せることを確認する。
7. 当該C3試行について人間が個別承認する。

C3床面60 mm評価へ進む場合は、上記とは別に固定カメラのArUco前後姿勢、床面経路、
斜行・実測距離の記録を準備し、新たなGO/NO-GO判定と個別承認を行う。

## C2中止条件

- ACK timeout、partial ACK、不正status、generation/sequence不一致
- Drive間隔500 ms以上または通信断
- host/Pico期限不一致
- 2.0秒超過、期限後Drive、STOP UART失敗
- 異常方向、異音、支持ずれ、再回転

異常試行では再ARM・再Driveを行わない。STOP後は二重ロックFalseへ戻す。

## 証拠

- `artifacts/roomba_trial_sessions/85267-1793344916-19096/latency.jsonl`  
  SHA256 `5d77b40353232eba2b8c2dea401ffc9c1ef2568b8788401aa9896bea349bd59c`
- `artifacts/roomba_trial_sessions/22758-1793349069-68/latency.jsonl`  
  SHA256 `5179c870a57e1b1ddb251ef530f2281cda438f05b600e2a8e32b656023a8c102`
- `artifacts/roomba_trial_sessions/27121-1793349573-16176/latency.jsonl`  
  SHA256 `e765a723a304fdcd223c7871b448c17fc9d720a28d30f4018218694321ee3938`
- `docs/C1_CONTINUOUS_DRIVE_RESULTS_20261030.md`
- `artifacts/roomba_trial_sessions/37181-1793350776-4836/latency.jsonl`  
  SHA256 `939484e054fce1e0177876481f8a0dcf1078bf5a5f3cf07828a18c65dcb1c377`
- `artifacts/roomba_trial_sessions/40861-1793351263-13771/latency.jsonl`  
  SHA256 `16136aa8ab3b328601ba16cfff1785dc40da4592b8697acee61d5290d6a63e78`
- `artifacts/roomba_trial_sessions/44041-1793351687-12691/latency.jsonl`  
  SHA256 `81d4552bcf1651e9ec5a8eeab571ed48c55fbc5d344ce6d48b6d425ff476d3b4`

SSDバックアップ: `/Volumes/名称未設定/robot_backup_20261030/artifacts/roomba_trial_sessions/`

