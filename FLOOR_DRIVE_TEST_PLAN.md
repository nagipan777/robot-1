# Roomba 680 段階的床面試験計画

更新日: 2026-10-20。無人／自律走行は **NO-GO**。有人限定Stage 2は
[ATTENDED_FLOOR_DRIVE_CHECKLIST.md](ATTENDED_FLOOR_DRIVE_CHECKLIST.md)の全項目がPASSの場合だけ、
試験責任者の明示承認後に単発実施できる。
本書は手順の準備だけを行い、走行ロック解除や実機操作を自動化しない。

## 固定安全条件

- 通常状態は `ALLOW_NONZERO_DRIVE=False`、`drive_enabled=false`、
  `control_enabled=false`、`calibration_ready=false`。
- Full modeは禁止。STOP ACK `0x07`、要求左右速度0、OI mode 2を満たさなければARMしない。
- 既存の実証済み上限は車輪浮上時の左右20 mm/s、最大1秒だけである。
  床面速度・時間・距離は未承認なので、本書では新しい数値を推測して設定しない。
- 操作者、監視者、物理隔離、独立停止手段のいずれかが欠けたら中止する。
- Pico電源断またはPico TX–Roomba RX断線ではSTOPを送れない。受容可能な物理対策が
  実証されるまで床面試験をGoにしない。

## Stage 0: 事前点検（走行なし）

1. 試験IDを発行し、写真、日時、担当者、床面状態を記録する。
2. バッテリーの膨張、発熱、液漏れ、電圧、充電状態を確認する。充電器を外す。
3. 無通電でUART1のGP4/GP5、BRC GP2、共通GND、レベル変換器、Roomba Mini-DINを確認する。
4. PicoのIP、9000 listener、protocol v2配備版、`ALLOW_NONZERO_DRIVE=False`を確認する。
5. ROS側の4安全フラグがfalseであることと、実機速度トピックのPublisherを確認する。
6. 読み取り診断3/3、STOP ACK、要求速度0、OI Safe、wheel drop、charging sourceを確認する。
7. 区域を囲い、段差、階段、人、動物、ケーブル、家具を除去する。
8. 監視者を1名置く。CLEAN/電源操作だけを独立停止保証とは扱わず、ロボットを安全に
   拘束・持ち上げられる配置と、電源を物理的に遮断できる手段を用意する。

開始前の読み取り専用確認:

```bash
git status --short
rg -n 'ALLOW_NONZERO_DRIVE = False' pico/pico_relay.py
rg -n 'declare_parameter\("drive_enabled", False\)' simulation/roomba_bridge_tcp.py
rg -n 'control_enabled: false|calibration_ready: false' simulation/vision_controller.real_dry_run.yaml
lsof -nP -iTCP:9000 -sTCP:ESTABLISHED
```

## Stage 1: 車輪浮上回帰

安定台、両輪完全浮上、監視者、即時STOPを確認した後に、既存Phase 4B手順だけを再利用する。

- 左右輪20 mm/s以下、1秒以下
- 通常STOPと物理停止
- 500 ms Pico watchdogとホスト側二次STOP
- watchdog後の`LATCHED_STOP`
- 同一世代・同一sequence・CRC不正指令の拒否
- 再接続だけでは再駆動しないこと
- 新しいARM世代後だけsequence 1を受理すること
- 試験終了後、両ロックをfalseへ戻しSTOP診断を再実行すること

一項目でもFAILならStage 2へ進まない。

## Stage 2: 有人・床面限定直進 — 条件付きHOLD

初回上限は、Phase 4Bで検証済みの左右各+20 mm/s、最大1.0秒を越えない。指令上の最大距離は
0.020 mである。1回の新規TCP sessionとARM generationで1回だけ直進し、連続試行しない。
正方向、左右輪符号、直進偏差、encoder 43/44差分、実測距離、停止距離、停止後の再始動有無を
同一試験IDで記録する。Navigation、YOLO、ArUco制御、他のcmd_vel Publisherは接続しない。

Pico電源断・UART抜線を走行中に試してはならない。全周物理境界、操作者、即時電源操作、
侵入防止が揃わなければHOLDのままとする。

## Stage 3: 限定旋回／無人自律 — NO-GO

Stage 2合格後に別承認する。時計回り・反時計回りを分離し、指令角、実測角、経過時間、
左右encoder差分、通常STOP、惰性、停止後挙動を記録する。トレッド幅0.235 mは暫定値であり、
実測・同定前に角度精度の合格値へ使わない。

## Stage 4: 停止後確認

- `LATCHED_STOP`中の旧世代非ゼロ指令がすべて拒否される。
- 通常STOP、診断、再接続ではラッチが解除されない。
- Safe喪失、wheel drop、通信断、例外で非ゼロ指令が再開しない。
- 新規ARM前に物理移動がない。
- 最終STOP ACK、要求速度0、OI Safeを確認して両ロックをfalseへ戻す。

## 記録

`floor_drive_test_log.py`は実機に接続せず、JSON全文とイベントCSVを同じ試験IDで保存する。

```bash
.venv/bin/python floor_drive_test_log.py \
  --test-id floor-20261020-001 --stage stage0 \
  --output artifacts/floor_drive_tests
```

必須イベントはARM generation、Drive sequence、左右速度、UART ACK、watchdog、STOP時刻・理由、
encoder生値/差分、実測距離/角度、目視停止結果。結果は`PASS`、`FAIL`、`NOT_RUN`のいずれかとし、
未測定値を0で埋めない。

## Go判定

有人Stage 2のGoには[ATTENDED_FLOOR_DRIVE_CHECKLIST.md](ATTENDED_FLOOR_DRIVE_CHECKLIST.md)の
全項目、Stage 1再合格、封じ込め対策と残留リスクの明示承認が必要。無人／自律走行は
[SAFETY_GO_NO_GO_CHECKLIST.md](SAFETY_GO_NO_GO_CHECKLIST.md)の従来条件を維持し、現在No-Goである。
