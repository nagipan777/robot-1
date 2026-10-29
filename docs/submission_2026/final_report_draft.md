# PAI最終課題 最終レポート初稿

## 1. 課題設定と目的

本課題は、固定カメラから対象物を検出し、Roomba 680の姿勢を推定し、ROS 2で移動候補を生成してPico W経由で実機へ伝える統合システムである。目的は単なる走行ではなく、認識喪失や通信異常を含む段階的な検証可能性を保ちつつ、AI判断が実機の限定低速動作へ到達することを示す点に置いた。

## 2. システム構成と技術選定

経路はUSB camera→YOLO/ArUco→ROS 2 Navigation→output adapter→TCP bridge→Pico W→Roomba OIである。YOLOはmaskを`litter`として検出し、ArUco DICT_4X4_50 ID 0からRoomba姿勢を得る。Navigation入力はArUco由来の`/robot/pose`であり、車輪odometryとは融合していない。

固定構成はv2 model（SHA256 `3e1de3...12b03`）、confidence 0.50、imgsz 960、camera 800×600である。較正は高さ0.53 m、80 mm marker、独立点誤差0.0427/0.0243 mで確認した。ただし有効範囲は固定環境の前方60～80 cmであり、一般環境性能を主張しない。[較正記録](../CAMERA_CALIBRATION_20261028.md)

## 3. 安全設計

実機出力は明示ARM世代、STOP latch、20 mm/s上限、host/Pico 400 ms session limit、500 ms watchdog、UART異常STOP、50 cm近傍guard、姿勢鮮度、人間の試行承認で制限する。BRC自動起床は使わずCLEAN手動起床とした。TCP接続を跨いだARM再利用、自動再ARM、Drive再送を禁止した。

これらは限定試験の防御であり、物理停止をソフトウェアだけで保証するものではない。各有人試行で人間が停止・異音・再発進を確認した。

## 4. 段階的な実験方法

検証は(1) unit/mock、(2) ROS replay/dry-run、(3)実カメラ非走行、(4)実Pico/Roomba非走行、(5)人間立会い単発の順に進めた。実カメラ非走行ではmask検出、ArUco姿勢、Navigation候補、検出途絶STOP、姿勢喪失STOP、同一目標の自動再開なしを確認した。Create3/Gazeboは認識・制御表示用のsimulationであり、実機結果と分離した。[証拠対応表](evidence_matrix.md)

## 5. 実機統合の結果

2026-10-28の有人床面単発では、AI経路からARMを経て左右20/20 mm/sをRoombaへ送った。PicoはDrive sequence 1–7を記録し、最初のDriveから通常STOPまで289 msだった。人間は短時間前進、両輪停止、異音なし、再発進なしを確認した。最終状態はSTOP ACK `0x07`、左右0、Safe 2、lock Falseである。[試行JSON](../../artifacts/step41/ai_attended_floor_trial_20261028.json)

2026-10-29の別試験はP3-2 deadline修正版の有人両輪浮上試験である。hostは6件の完全ACKを受信し、7件目のACK待ちをdeadlineで中断してSTOP・ARM失効・接続破棄を行った。Picoは期限前に届いた7件目を含む7 Driveを記録し、最初のDriveから通常STOPまで224 ms、STOP後Drive 0件だった。物理停止と再発進なしも確認した。[P3-2報告](../../P3_2_DEADLINE_IO_SEPARATION_REPORT_20261029.md)

## 6. ARM ACK消失の原因と修正

初回短縮EではPicoがARM generationを受理した一方、hostは0/7 byteでtimeoutした。MicroPythonのnon-blocking `socket.send()`が`None`を返した際、旧実装が残り全byteを送信済みと誤認したことが根因だった。

修正は、would-block/`None`を既存送信期限内で再試行し、全byte送信後だけARMをcommitするものとした。`ARM_PENDING`、connection ID、失敗時STOP latch、古いgeneration拒否も追加した。実Pico locked非走行probeは10/10成功した。timeout延長では解決していない。[ARM/TCP分析](../../ARM_TCP_RECOVERY_20261026.md)

## 7. deadline遅延の原因と修正

旧20 Hz timerでは期限検知が約41 ms遅れた。100 Hz化後も実通信でsingle-threaded executorが同期Safe照会またはDrive ACK待ちにより約181 ms塞がれた。STOP TCP write自体は0.483 msで主因ではなかった。

P3-2ではsocket所有者を一元化したまま、固定応答待ちを最大10 msのpollへ分割した。絶対期限では同じ所有者がSTOPを送信し、ARMを失効してsocketを閉じる。遅延ACKは次セッションへ持ち越さず、未送信Driveは破棄する。50/181/300 ms遅延、部分受信、切断、遅延ACKを模擬し、focused 66/66、全体369/369がPASSした。

有人浮上試験ではこの中断経路が動作した。ただし`latency_trace_path`未指定のため厳密なhost STOP開始時刻は保存されていない。開始から後続エラーログまでは400.734 msで、STOP送信・closeはそれ以前である。後のログ改善版は回帰PASSだが追加実機走行なしである。よって「Pico相対Drive→STOP 224 ms」は実測、「host STOP begin <=400.000 ms」は未確認と区別する。

## 8. 観測機能、限界、今後

車輪odometryはencoder CSV replayから`/roomba/odom`を発行する観測用dry-runを完成させた。wrap、jump、途絶、再接続を検証し、TFとNavigation接続は無効である。OI packet 43/44の実機取得、左右符号、scale、実時間比較は未統合であり、実機odometry PASSとはしない。[odometry報告](../../ODOMETRY_DRYRUN_REPORT_20261027.md)

成果は、限定された固定環境でAI認識から実機低速単発まで到達し、実際に生じたARM ACK消失とdeadline阻害を原因ベースで修復した点にある。残る課題は、実機動画の収録、厳密host STOP begin trace、一般環境の検出評価、実機encoder統合、連続運転評価である。これらを未確認のまま無人連続走行の安全性は主張しない。

提出準備時点の最新全回帰は373/373 PASSである。本文中の369/369はP3-2修正時点の記録であり、後続の文書・試験追加後に373件へ増えた。公式提出形式はリポジトリ内で確認できておらず、最終提出前に外部の課題要項と照合する必要がある。

## 参考資料

- [証拠対応表](evidence_matrix.md)
- [最終統合チェックリスト](final_integration_checklist.md)
- [動画台本](video_storyboard.md)
- [素材一覧](asset_inventory.md)
- `AI_ATTENDED_FLOOR_DEADLINE_REPORT_20261029.md`
- `P3_2_DEADLINE_IO_SEPARATION_REPORT_20261029.md`

