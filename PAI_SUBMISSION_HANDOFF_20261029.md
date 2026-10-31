# PAI最終課題 提出資料整理・レビュー依頼

更新日: 2026-10-29
対象ブランチ: `fix-arm-ack-session-recovery`
GitHub HEAD: `bb48c0a4d1acae22debbd432b3d6ee0d994780aa`
PR: #3（OPEN、未マージ）

## 1. 提出時の一文要約

YOLOによるマスク検出、ArUcoによるRoomba姿勢推定、ROS 2 Navigation候補、
TCP経由のPico W安全中継、Roomba 680の低速駆動までを統合した。通信断・ACK
消失・入力途絶に対し、ARM世代管理、STOPラッチ、20 mm/s上限、ホスト/Pico
400 msセッション制限、500 ms watchdogを維持し、有人単発試験で短時間駆動と
物理停止を確認した。

## 2. 最終成果の判定表

| 項目 | 最終判定 | 証拠 |
|---|---|---|
| YOLO実カメラ検出 | 条件付きPASS | v2モデル、confidence 0.50、imgsz 960、前方60～80 cmの管理範囲 |
| ArUco ID 0姿勢 | PASS | 実カメラで継続認識、map座標の姿勢候補を約9.8 Hzで確認 |
| AI→Navigation非走行E2E | PASS | 目標採用、非ゼロ候補、検出途絶STOP、同一目標の自動再開禁止 |
| AI連動実機経路 | PASS（有人単発） | YOLO→ArUco→Navigation→ARM→Pico→Roomba、Drive ACK、物理停止 |
| ARM ACK/TCP再接続 | PASS | `send(None)`誤認修正、ARM_PENDING、接続ID、実Picoロック診断10/10 |
| STOP/Safeゲート | PASS | STOP ACK `0x07`、左右要求速度0、OI Safe mode 2 |
| 20 mm/s速度上限 | PASS | 実機Drive ACKおよびPicoログで左右20/20 mm/s |
| ブロッキングI/O中のdeadline STOP | PASS | ACK待ちを期限で中断し、STOP送信、ARM失効、セッション破棄 |
| Pico実測Drive→STOP | PASS | 最初のDrive `392768 ms`、STOP `392992 ms`、224 ms |
| 期限後Drive・再ARM | PASS | STOP後Drive 0件、自動再ARM・再Driveなし |
| 物理停止 | PASS | 両輪停止、異音なし、再発進なしを人間が確認 |
| 観測専用オドメトリ | dry-run PASS | `/roomba/odom`、CSV、周回・jump・途絶・再接続の模擬検証 |
| 実機エンコーダ取得 | 未確認 | Navigationへ未接続、提出必須機能にはしない |
| 全回帰 | PASS | 最新369/369 |

## 3. 障害と修復の時系列

### ARM ACK消失

初回短縮EではPicoがARMを受理した一方、MacはACKを0/7 byteしか受信できず、
READY timeoutと再接続不良が続いた。原因はMicroPythonの`socket.send()`が
`None`を返した場合を送信完了と誤認していたことと、ACK完全送信前のARM確定で
あった。全byte送信、ACK送信完了後のcommit、接続ID、失敗時STOPラッチへ修正し、
実Picoのロック診断と再接続10/10を確認した。

### 400 ms期限超過

旧20 Hzホスト処理ではSTOPが約43 ms遅延し、その後の100 Hz版でも同期Safe照会
またはDrive ACK待ちがsingle-threaded executorを約181 ms塞いだ。TCP STOP書込み
自体は主因ではなかった。

P3-2では、単一のTCP所有者を維持したまま固定応答待ちを最大10 ms単位で監視し、
絶対期限で同じ所有者がSTOPを送信、ARMを失効し、遅延ACKを再利用しないよう接続を
破棄する方式へ変更した。50/181/300 msの遅延、部分受信、切断、遅延ACKを模擬し、
期限後Drive 0件と応答取り違えなしを確認した。

有人両輪浮上試験では、ACK待ちをdeadlineで中断してSTOPを送信した。Pico側では
最初のDriveから224 msで通常STOPを受け、STOP後Driveは0件だった。

## 4. 正確に残す制約

- YOLOのPASS範囲は、固定カメラ・較正済み環境・前方60～80 cmである。一般環境の
  物体検出性能を主張しない。
- ホスト試験ではセッション開始から後続エラーログまで400.734 msだった。STOP送信
  とsocket closeはそのログより前だが、当該試行は`latency_trace_path`未指定のため、
  ホストSTOP開始の厳密な小数msは保存されていない。
- 上記の観測欠落後、通常例外ログへdeadline、STOP開始、超過量、STOP送信時間を
  出す修正を追加し369/369回帰PASSとしたが、そのログ改善版で追加走行はしていない。
- 実機エンコーダとオドメトリは未接続であり、Navigation入力はArUco由来の姿勢を
  維持した。
- BRC自動起床は使用せず、CLEAN手動起床を採用した。
- 本成果は有人単発試験の証拠であり、無人連続走行の許可ではない。

## 5. 提出資料で推奨する構成

1. **目的**: AI認識から実機低速動作までの安全な統合。
2. **システム構成図**: USBカメラ→YOLO/ArUco→ROS 2→TCP bridge→Pico W→Roomba。
3. **安全設計**: ARM、STOPラッチ、20 mm/s、400 ms、500 ms watchdog、50 cmガード。
4. **失敗と改善**: ARM ACK消失とexecutorブロッキングを主要な技術課題として説明。
5. **検証**: mock/dry-run、実Pico非走行、AI非走行、有人単発の順に提示。
6. **成果**: AI連動実機の有人単発成功と、別のdeadline修正浮上試験での224 ms
   STOP・再発進なしを、混同せず提示。
7. **制約と今後**: 管理された検出範囲、実機オドメトリ未統合、連続走行未検証。

## 6. 動画で最低限見せる場面

1. 固定カメラ映像でマスクの`litter` bboxとArUco ID 0を同時表示。
2. ターミナルでSTOP ACK `0x07`、左右0、Safe mode 2を表示。
3. 人間の試行承認とARM generationを表示。
4. 20 mm/sの短時間駆動と自動STOPを一続きで撮影。
5. STOP後に両輪停止・再発進なしを数秒見せる。
6. 最後に`ALLOW_NONZERO_DRIVE=False`と実機出力停止を表示。

過去映像を使う場合、後から追加したログ改善版で撮影したように編集しない。映像と
対応するコミット・ログの版を明記する。

## 7. 提出前に残る作業（優先順）

### 必須

- レポート本文と動画の数値を本書の最終値へ統一する。
- 旧報告書の「P3 NO-GO」「400 ms FAIL」が修正前の結果であることを時系列で示す。
- GitHub READMEまたは提出PDFから、最終報告書とPR #3へ到達できるようにする。
- 動画・PDF・GitHubのモデル名、SHA256、confidence、imgszを一致させる。

### 余力があれば

- 通常ログ改善版のSTOP時刻を追加の非走行モック出力例として掲載する。
- 観測専用オドメトリのdry-run画面を補足資料へ入れる。
- 実機エンコーダは追加走行せず、未確認事項として図示する。

### 提出後へ延期可能

- 実機オドメトリ融合。
- BRC自動起床。
- 一般環境・長距離・連続走行への拡張。
- モデルの追加学習と広範なholdout評価。

## 8. チャッピーへのレビュー依頼文

以下をそのまま渡せる。

> PAI最終課題の提出資料をレビューしてください。安全機構を維持したまま、YOLO・
> ArUco・ROS 2・Pico W・Roomba 680を統合しました。ARM ACK消失とホストdeadline
> 遅延を発見し、修正・回帰・非走行・有人両輪浮上の順で検証しています。
> 最終実測では左右20/20 mm/s、Pico上の最初のDriveからSTOPまで224 ms、STOP後Drive
> 0件、物理停止・再発進なしです。一方、当該試行ではホスト内部STOP開始の厳密な
> 小数msをtraceへ保存しておらず、後続エラーログはセッション開始から400.734 ms
> でした。ログ改善コードは回帰PASSですが追加走行していません。YOLOの保証範囲は
> 固定環境の前方60～80 cmで、実機オドメトリは未統合です。
>
> 次の観点で意見をください。
> 1. 「AI連動実機統合PASS」と「厳密なホストSTOP開始時刻未保存」の表現は妥当か。
> 2. 追加走行なしで提出する場合、どの証拠を主資料・補足資料へ分けるべきか。
> 3. 旧FAIL→原因特定→修正→再検証の流れは評価されやすい構成になっているか。
> 4. 誇張、矛盾、説明不足になっている箇所はどこか。
> 5. 11月2日提出に向け、削るべき内容と必ず残す内容を優先順位付きで示してほしい。

## 9. 参照資料

- `P3_2_DEADLINE_IO_SEPARATION_REPORT_20261029.md`
- `AI_ATTENDED_FLOOR_DEADLINE_REPORT_20261029.md`
- `docs/AI_NONDRIVING_GATE_20261028.md`
- `P1_ROOMBA_NONDRIVING_REPORT_20261027.md`
- `WIFI_P0_REPORT_20261027.md`
- `PAI_INTEGRATION_PROGRESS_REPORT_20261027.md`
- `ODOMETRY_DRYRUN_REPORT_20261027.md`
- `WHEEL_MOTION_MONITOR_REPORT_20261029.md`
