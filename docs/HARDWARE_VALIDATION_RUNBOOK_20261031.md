# 実機確認手順書（2026-10-31 単独オペレーター向け）

## 1. この手順書の目的

この手順書は、2026年10月31日の安全系修正をPico Wへ反映し、Roomba 680が次の状態になることを人間の立会いで確認するためのものです。

- 接続直後に停止命令が届く
- 左右の要求速度が0になる
- RoombaがOI Safe mode 2になる
- 明示的な許可（ARM）なしでは回転しない
- 許可された短時間試験後に停止し、勝手に再回転しない

これは「すべての安全性が実機で証明された」という試験ではありません。今回の3修正は非走行回帰テスト380件で確認済みですが、修正版コードそのものの実機確認はまだ行われていません。

## 2. 想定する操作者

この手順書は、対話支援を使わず、操作者1名で完結できるように記載しています。想定するスキルは次のとおりです。

- macOSのターミナルで、記載されたコマンドを実行できる
- `cd`、`git status`、`git rev-parse`、`Ctrl+C`を理解している
- MicroPicoまたはThonnyでPico上の `main.py` を編集・保存・実行できる
- ログ中の指定文字列を照合できる
- 配線確認と独立電源遮断を実施できる

安全確認、試験開始の判断、物理停止の確認は操作者が行います。コマンドを連続して貼り付けず、各STEPの結果を確認してから次へ進んでください。

## 3. 用語（画面に出ても慌てないための説明）

| 表示 | 意味 |
|---|---|
| STOP | 車輪速度を左右とも0にする要求 |
| ACK | PicoまたはRoombaが要求を処理したことを示す応答 |
| `0x07` | STOPが正常に処理されたことを示す応答 |
| ARM | 非ゼロの車輪指令を1回の試験だけ許可する鍵 |
| Safe mode 2 | Roombaの安全制限が有効なOIモード |
| watchdog | 指令が500ms途絶えた場合に停止する既存機構 |
| STOPラッチ | STOP後、再ARMするまで非ゼロ指令を拒否する状態 |

## 4. 今回変更してはいけない値

Picoの `main.py` で次の4行を確認します。数字や真偽値を推測で変更してはいけません。

```python
COMMAND_TIMEOUT_MS = 500
DRIVE_SESSION_TIMEOUT_MS = 400
ALLOW_NONZERO_DRIVE = False
BRC_WAKE_ENABLED = False
```

通常の安全状態は `ALLOW_NONZERO_DRIVE = False` です。`True` にするのは、すべての事前確認が終わり、人が当該試行を承認した後だけです。試験終了後は必ず `False` に戻します。

## 5. 即時中止条件

次のどれか1つでも起きたら、その試行を終了します。自動再ARMや同じ試行内での再Driveは行いません。

- 左右どちらかの車輪が意図しない方向へ回る
- 400msを明らかに超えて回り続ける
- STOP後に再回転する
- 支持台がずれる、配線が張る、発熱、膨張、焦げ臭さ、異音がある
- `STOP ACK 0x07`、左右0、Safe mode 2のどれかを確認できない
- ACK timeout、部分応答、TCP切断、CRC異常、不正ACKが出る
- ブリッジが2個以上動いている疑いがある
- Picoの実配備コードが確認済みコミットと一致しない

中止時は、まず独立停止手段を使います。車輪が停止したことを目視確認してから、ターミナルを整理します。

## 6. STEP 0：コードを固定する（実機には触れない）

使用するコードは次です。

```text
branch: fix-arm-ack-session-recovery
commit: bc175d4243c43baee0684ff29b94cb479061ea38
```

既存の作業フォルダーに未コミット変更がある場合、そこで `git checkout` や `git reset` を実行しないでください。別のクリーンなworktreeを準備します。

```bash
cd /Users/imaichika/robot
git fetch origin fix-arm-ack-session-recovery
git worktree add /private/tmp/robot-hardware-validation-20261031 bc175d4243c43baee0684ff29b94cb479061ea38
cd /private/tmp/robot-hardware-validation-20261031
```

すでに同名worktreeが存在する場合は、削除や上書きをせず、その状態を確認してから別名を使用してください。

コミットを確認します。

```bash
git rev-parse HEAD
```

作業ツリーを確認します。

```bash
git status --short
```

### PASS条件

- `git rev-parse HEAD` の結果が上記40文字と完全一致する
- `git status --short` が空である
- 全回帰テストが `380 passed` である

一致しない場合は実機へ配備しません。

以降のコマンドは、特記がない限り次のフォルダーで実行します。

```text
/private/tmp/robot-hardware-validation-20261031
```

## 7. STEP 1：RoombaとPicoを無通電で準備する

1. Pico WのUSBを抜きます。
2. RoombaのCLEAN表示が消えていることを確認します。
3. BRC（Roomba Pin 5からPico GP2）は接続しません。
4. GP4 TX、GP5 RX、GND、レベル変換器の配線を目視確認します。
5. Roombaを支持台へ載せ、左右両輪が床や物に触れていないことを確認します。
6. USBをすぐ抜ける位置、または独立電源を切れる位置に置きます。

### PASS条件

- 両輪完全浮上
- 支持台が安定
- 配線に張力・干渉なし
- BRC未接続
- 独立停止手段をすぐ使用可能

## 8. STEP 2：確認済みのPicoコードを配備する

MicroPicoまたはThonnyを使います。

1. リポジトリの `pico/main.py` を開きます。
2. Pico上の `main.py` を開きます。
3. リポジトリ版の内容でPico版を上書きします。
4. Wi-FiのSSIDとパスワードだけをPico上のファイルへ記入します。
5. Wi-Fi認証情報をリポジトリ側へ保存しません。
6. 4章の4行が完全一致することを確認します。
7. Pico上の `main.py` として保存します。
8. 実行または再起動します。

### 期待する表示

```text
Wi-Fi OK; IP: （PicoのIPアドレス）
listening on （PicoのIPアドレス）:9000
```

### 異常時

- Tracebackが出た場合は続行しません。
- IPアドレスが表示されない場合は続行しません。
- 車輪が動いた場合はUSBを抜き、物理停止を確認します。

## 9. STEP 3：Roombaを手動起床する

1. PicoがTCP待受中であることを確認します。
2. RoombaのCLEANボタンを1回押します。
3. CLEAN表示の点灯を確認します。
4. 左右両輪が停止したままか5秒間確認します。

### PASS条件

- CLEAN表示が点灯
- 両輪停止
- 異音なし
- 再回転なし

## 10. STEP 4：STOP専用の静止診断を行う

Pico画面に表示されたIPを使って次のコマンドを実行します。以下の例ではIPを `192.168.0.32` としています。実際の表示が異なる場合は、そのIPへ置き換えてください。

```bash
./simulation/sim.sh roomba-tcp-diagnostic 192.168.0.32 9000
```

`PICO_IP` のようなプレースホルダーを文字のまま入力する手順ではありません。

### 期待するログ

```text
Connected to Pico W ...; relay READY; STOP sent
STOP confirmed: ... roomba_requested_right=0, roomba_requested_left=0, ack=0x07
Roomba OI mode: 2 (Safe)
```

### PASS条件

- READYを受信
- STOP ACKが `0x07`
- right=0、left=0
- OI modeが2
- 車輪が停止したまま

初回timeout後に再接続で成功した場合、通信が復旧した事実は記録しますが、初回timeoutをなかったことにはしません。次の走行試験へ進むには、停止状態から開始した1セッションで3条件が揃うことが必要です。

確認後、`Ctrl+C` を1回押します。シェルへ戻り、車輪停止を確認します。

## 11. STEP 5：非ゼロ試験の直前確認

ここから先は、操作者がチェック欄をすべて満たし、当該試行の実施を明示的に決定した場合だけ進みます。

確認項目：

- 両輪完全浮上
- 支持台安定
- 配線干渉なし
- 独立停止手段使用可能
- 両輪停止、異音なし
- Pico IP確認済み
- STOP ACK `0x07`、左右0、Safe 2確認済み
- `roomba_bridge_tcp` を起動しているターミナルが0個
- `ALLOW_NONZERO_DRIVE=False` から開始している

稼働中プロセスを確認します。

```bash
docker --context colima-create3 compose -f simulation/compose.yaml exec -T simulator pgrep -af roomba_bridge_tcp.py
```

何も表示されなければ、既存ブリッジは0個です。1行以上表示された場合は新しいブリッジを起動せず、所有しているターミナルから正常終了してください。無関係なプロセスを一括終了しないでください。

全条件が成立したら、試験記録へ次のように記入してから進みます。

```text
修正版コード・両輪浮上・左右20mm/s以下・最大400msの有人単発試験：実施承認
承認者：
時刻：
```

## 12. STEP 6：走行ロックを試行中だけ解除する

1. Picoの `main.py` を停止します。
2. `ALLOW_NONZERO_DRIVE = True` へ変更します。
3. それ以外の行を変更していないことを確認します。
4. 保存します。
5. 再起動します。
6. Wi-Fi接続、IP、TCP `:9000` 待受を確認します。
7. 両輪が停止したままか確認します。

この `True` は安全性確認済みの恒久設定ではなく、この1回だけの試験設定です。

## 13. STEP 7：単一ブリッジを起動する

ターミナルAを開き、作業フォルダーへ移動します。

```bash
cd /private/tmp/robot-hardware-validation-20261031
```

続けて、実際のPico IPを指定して起動します。

```bash
./simulation/sim.sh roomba-short-e-bridge 192.168.0.32 WHEELS_LIFTED 9000
```

このコマンドには既存の重複起動拒否、20mm/s上限、500ms指令timeout、400ms走行上限が含まれます。

### GO条件

ターミナルAに次が1回の接続内で表示されること：

- `relay READY; STOP sent`
- `STOP confirmed ... ack=0x07`
- `requested_right=0, requested_left=0`
- `Roomba OI mode: 2 (Safe)`

重複ブリッジ拒否、timeout、Broken pipe、再接続ループが出た場合はNO-GOです。

## 14. STEP 8：400msの単発回転試験を実行する

ターミナルBを開き、作業フォルダーへ移動します。

```bash
cd /private/tmp/robot-hardware-validation-20261031
```

次を実行します。

```bash
./simulation/sim.sh roomba-wheel-lift-test rotate WHEELS_LIFTED
```

注意：コマンド名は `rotate` ですが、現在の既定値は右20mm/s・左20mm/sです。この試験で期待する物理動作は、左右両輪が同じ前進方向へ短時間回転することです。旋回試験ではありません。

### 人が目視すること

- 左右両輪が同じ前進方向へ回る
- 約400msで停止する
- 異音や支持台のずれがない
- 停止後5秒以上、再回転しない

### ログで確認すること

- ARM accepted
- 同じARM generationでDrive ACKを受信
- right=20、left=20を超えない
- STOPが送信される
- STOP後に非ゼロDriveがない

### PASS条件

ログと目視の両方が一致した場合だけPASSです。Drive ACKだけでは物理動作PASSにしません。

## 15. STEP 9：終了と安全状態への復帰

順番を変えません。

1. ターミナルBが終了していることを確認します。
2. ターミナルAで `Ctrl+C` を1回押します。
3. シェルへ戻ったことを確認します。
4. 左右両輪が停止し、5秒以上再回転しないことを確認します。
5. Picoの `main.py` を停止します。
6. `ALLOW_NONZERO_DRIVE = False` へ戻します。
7. 他の3安全設定が4章と一致することを確認します。
8. 保存して再起動します。
9. Wi-Fi接続とTCP待受を確認します。
10. STEP 4のSTOP専用診断をもう一度実行します。
11. STOP ACK `0x07`、左右0、Safe 2を確認します。
12. 診断を `Ctrl+C` で終了します。
13. Picoの実行を停止します。
14. PicoのUSBを抜きます。
15. Pico LED消灯、両輪停止、異音なし、再回転なしを確認します。

## 16. 結果の記録

次を試験記録へ残します。

- 使用コミット：`bc175d4243c43baee0684ff29b94cb479061ea38`
- PicoのIP
- 試験開始・終了時刻
- STOP ACK
- OI mode
- ARM generation
- Drive sequenceとACK件数
- 左右要求速度
- STOP時刻
- 車輪の回転方向
- 物理停止と再回転の有無
- timeout、切断、再接続の有無
- 試験後 `ALLOW_NONZERO_DRIVE=False` の確認

ターミナルログとPicoコンソールは別々に保存します。古いログを上書きしません。

試験記録には、少なくともターミナルA、ターミナルB、Picoコンソールの3種類を保存します。ログ取得方法を試験直前に変更すると動作条件が変わるため、未検証の `tee` や新しいラッパーを本番で追加しないでください。ターミナルの画面保存または既存の端末ログ機能を使用します。

## 17. 3修正について、この手順で分かること／分からないこと

| 修正 | この基本手順で確認できること | この基本手順だけでは未確認のこと |
|---|---|---|
| 速度ゼロ量子化 | 通常のDrive ACKとSTOPに退行がない | 微小な非ゼロTwistが左右0へ丸められる専用ケース |
| 診断STOPでARM失効 | 起動時診断STOPと通常試験が成立する | ARM後に診断STOPを送り、同世代の次Driveが実機で拒否される専用ケース |
| UART待ち中の期限監視 | UART正常応答時の通常試験に退行がない | 駆動中にUART無応答を故意に発生させる故障注入ケース |

上記3つの未確認項目を、この通常試験でPASSと記録してはいけません。これらは自動状態遷移テストでは確認済みですが、実機で直接確認するには、単一TCP所有者を保つ専用ハーネスが必要です。

### 行ってはいけない代替操作

- 別のターミナルからPicoへ2本目のTCP接続を作る
- 動作中に思いつきでROS topicを手入力する
- 駆動中にUART線を抜いて故障を作る
- timeout値や400ms／500msを延長する
- ACK不明のDriveを再送する

これらは再現性を下げ、どの経路を検証したか分からなくするためです。

## 18. 最終判定の書き方

結果は次の4区分で記録します。

- **PASS**：ログと目視で確認済み
- **FAIL**：期待動作と異なる事実を再現
- **UNVERIFIED**：今回の試験範囲に含めていない
- **BLOCKED**：配線、通信、承認など前提不足で実施できない

「非走行テストPASS」「通常実機回帰PASS」「故障注入PASS」は別々の成果です。通常実機回帰がPASSしても、3修正の個別故障注入を実機確認済みとは記載しません。
