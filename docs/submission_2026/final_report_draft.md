# AIによるゴミ認識に基づく Roomba 680の低速移動制御

作成者：nagisan1012　更新：2026年10月31日

## 1 タスクの目的と達成した行動

本課題のタスクは、床面のゴミを固定カメラで認識し、対象位置とRoomba 680の姿勢から移動指令を生成して、有人監視下で低速駆動・停止させることである。清掃作業への応用を目指し、認識・幾何変換・制御・通信を接続するPhysical AIシステムを構築した。

固定環境でマスクをlitterとして検出し、YOLO・ArUco・ROS 2 Navigation・Pico Wを介した実機の短時間前進と停止に成功した[1]。別の床面直進試験では56 mmの実移動を確認した[5]。ただし、その56 mmをAIによる目標到達の結果とは扱わず、ゴミへの自律接近・回収完了は今後の課題とする。

## 2 AIと通常制御の役割分担

学習モデルを使うAI部分はYOLOである。ArUco・床面射影は幾何計算、Navigationと実機出力は通常の制御・通信処理である。ARMは走行許可の状態であり、ロボットアームを意味しない。

| 機能 | 担当する処理 | 出力 |
| --- | --- | --- |
| YOLO | 画像からマスクをlitterとして物体検出[10] | Bounding Box・信頼度 |
| ArUcoと床面射影 | マーカー検出と較正値から姿勢・対象位置を計算[12] | map座標の位置・向き |
| ROS 2 Navigation | 承認目標と現在姿勢から旋回・前進速度を計算 | 速度候補 |
| ARM・Pico W・Roomba | 走行許可と速度制限を検査し、TCP→UART/OIで駆動 | 車輪の低速動作・停止 |

構成：固定カメラ → YOLOによる対象検出／ArUco・床面射影による位置・姿勢 → ROS 2 Navigation → 走行許可ARMとTCP → Pico W → UART/OI → Roomba 680。

## 3 学習と実環境に合わせた工夫

TACO[11]による初期学習後、Roomba本体や床テープの誤検出を発見した。マスク正例と空ラベルのRoomba負例を追加し、位置・向きの異なる実画像を混合してv2を選定した[1,2]。hard-negative段階の分割はtrain 53枚・val 20枚・test 19枚であり、最終v2の全学習枚数とは区別する。

実機統合試験ではv2、confidence 0.50、推論imgsz 960、カメラ取得800×600を使用した。報告値は混合test mAP50 0.641、実画像test 0.982である[1]。小規模・同環境での値を未知環境に一般化しない。28日に確認した前方60〜80 cmも、29日には70 cmで検出が不安定となり、配置ごとの再現性を残した。

固定カメラ較正の当てはめRMSEは0.0193 m、独立2点の誤差は0.0427 m・0.0243 mだった[3]。これらは平面較正の誤差であり、実機移動中の総合精度や3D自己位置精度を直接示す値ではない。

## 4 実験結果とタスク達成範囲

認識から実機駆動までの統合結果を主成果とし、直進の継続性や停止期限は別の評価項目として記録した。試行ごとにソフトウェア版、床面／浮上、速度・期限が異なるため、結果を一つの連続成功として合算しない。

| 実験 | 達成・観測した行動 | 評価と限界 |
| --- | --- | --- |
| 10月28日 AI連動・有人床面 | 左右20 mm/s、Drive ACK 7件。Pico上のDrive→通常STOP 289 ms。短時間前進・物理停止・再発進なし[1]。 | AI連動実機統合に成功。目標到達・回収は未評価。 |
| 10月29日 期限制御・浮上 | ACK待機を期限で中断。Pico上のDrive→STOP 224 ms、STOP後Drive 0件。物理停止を確認[4]。 | 停止動作を確認。ホスト400 ms厳守は未判定。 |
| 10月29日 AI統合の再試行 | ARMは受理されたが、新鮮なNavigation候補がなくDrive 0件・移動なし。再ARMせず終了。 | 再試行は不成立。候補鮮度とARM操作の同期が課題。 |
| 10月30日 C3有人床面直進 | 1回目26 mm。Safe照会の動作窓分離後、2回目はACK 22件・実測56 mm。watchdog停止、異音・再発進なし[5]。 | 距離40〜80 mm帯は達成。2.8秒の連続性と試行総合はFAIL。 |
| 10月31日 C2再構成版・浮上 | 2.0秒上限で3試行。1回目ACK 29件、初回ACK→STOP約1.831秒。2・3回目は未完了。全3回で停止・再発進なし[6]。 | 動作完了1/3、停止3/3。連続性は2/3不成立で、総合再現性は未達。 |

## 5 障害を切り分けて改善した点

ARM ACK消失は、MicroPythonのsend()がNoneを返した際の送信完了誤認を修正し、ACK全送信後だけARMを確定する方式にした。接続IDと世代管理を加え、実Picoのロック状態・非走行再接続試験は10/10成功した[9]。

連続制御では、同期Safe照会が送信を妨げたため、専用試験モードの動作窓から分離した[5]。ACK送信完了と相手への到達も区別した。Macネイティブの非走行30応答は全て受信したが、最大RTTは267.574 msだった。TCP再送は観測されず、無線・キューの影響は候補として残る[7]。31日の再試験では別に、ACK受信後のホスト処理停滞とOI Safe喪失も確認され、原因を一つに決めつけなかった[6]。

## 6 停止期限の評価と位置付け

400 msは本システムで設定した追加の停止性能評価であり、タスク実行の成否とは分けて示す。旧実装ではSTOP送信開始が441.812 ms、100 Hz監視追加後の別試行でも583.136 msとなった。同期I/Oを期限確認付き待機へ変更し、期限時にSTOP・ARM失効・接続破棄を行う方式に修正した[4]。

修正版ではPico側224 msの停止を確認したが、ホスト内部のSTOP開始時刻を保存しておらず、厳密な400.000 ms以内の開始は未判定である。後続エラーログの400.734 msをSTOP開始時刻に置き換えない。2.0／3.0秒の連続試験は通常400 msモードと分離し、500 ms watchdogは維持した[4–6]。

## 7 考察と今後の課題

本課題では、AIによる対象認識を実機の限定動作へ接続し、実環境の誤検出、ACK消失、同期I/Oによる時間制御の阻害を観測・修正した。56 mmの床面移動や一度の連続動作を達成した一方、繰り返し動作の再現性は未達である。実測した行動、模擬検証、未検証の範囲を区別することが、成果を正確に示す上で重要だった。

今後は、候補の生成時刻とARM受理の同期、ホスト処理・無線遅延の切り分け、厳密なSTOP開始時刻の計測、目標到達・停止位置の評価を進める。観測用オドメトリは模擬ROS 2 E2Eまでで、実機OI packet 43/44取得とNavigationへの融合は未統合である。10月31日の通常経路の安全修正も非走行回帰380/380であり、修正後の実機合格とは扱わない[8]。

## 8 動画とコード一式

提出動画PAI_test3.mp4は、対象認識、システム構成、実機、課題の説明を含む。認識→姿勢・移動候補→実機の行動というタスクの流れを中心に示し、STOPの詳細は補足として説明する。Create 3の記録軌跡再生はシミュレーションと明示し、編集映像を一連の自律走行や期限測定の証拠にしない。

コード一式は https://github.com/nagipan777/robot-1 に公開し、提出原稿はfeature/navigation-hardware-unit-test、初期統合・期限修正の証拠はfix-arm-ack-session-recoveryを参照する。試験で用いた版と後の修正版を区別し、C2の再構成版を当時の配備ソースそのものとは主張しない。

## 9 参考文献と検証資料

実機試験モデル：best_position_v2_20261028.pt。SHA-256：

3e1de3c72c6d807fd527cefc6b70801275e3a9b54bb9daa8d87a59dc7be12b03

[1] AI非走行ゲートとv2評価・有人床面試験. 2026-10-28. https://github.com/nagipan777/robot-1/blob/fix-arm-ack-session-recovery/docs/AI_NONDRIVING_GATE_20261028.md

[2] 実環境hard-negative追加学習報告. 2026-10-28. https://github.com/nagipan777/robot-1/blob/fix-arm-ack-session-recovery/docs/YOLO_HARD_NEGATIVE_20261028.md

[3] 固定カメラ較正記録. 2026-10-28. https://github.com/nagipan777/robot-1/blob/fix-arm-ack-session-recovery/docs/CAMERA_CALIBRATION_20261028.md

[4] P3-2期限修正と実機検証報告. 2026-10-29. https://github.com/nagipan777/robot-1/blob/fix-arm-ack-session-recovery/P3_2_DEADLINE_IO_SEPARATION_REPORT_20261029.md

[5] C3_CONTINUOUS_FLOOR_RESULTS_20261030.md. 実装班提出補足資料. 2026-10-30.

[6] C2再検証報告. 2026-10-31. https://github.com/nagipan777/robot-1/blob/feature/navigation-hardware-unit-test/docs/C2_REVALIDATION_REPORT_20261031.md

[7] C3_TCP_ACK_DELIVERY_INVESTIGATION_20261030.md. 実装班提出補足資料. 2026-10-30.

[8] 非走行安全修正報告. 2026-10-31. https://github.com/nagipan777/robot-1/blob/fix-arm-ack-session-recovery/docs/NON_DRIVING_FIXES_20261031.md

[9] ARM ACK通信障害と実Pico再検証. 2026-10-27. https://github.com/nagipan777/robot-1/blob/fix-arm-ack-session-recovery/NIGHTLY_ARM_TCP_REPORT_20261027.md

[10] Redmon, J. et al. You Only Look Once: Unified, Real-Time Object Detection. CVPR, pp. 779–788, 2016. https://arxiv.org/abs/1506.02640

[11] Proença, P. F. and Simões, P. TACO: Trash Annotations in Context for Litter Detection. arXiv:2003.06975, 2020. https://arxiv.org/abs/2003.06975

[12] Garrido-Jurado, S. et al. Automatic generation and detection of highly reliable fiducial markers under occlusion. Pattern Recognition, 47(6), pp. 2280–2292, 2014. https://doi.org/10.1016/j.patcog.2014.01.005

[13] Casini, D. et al. Response-Time Analysis of ROS 2 Processing Chains Under Reservation-Based Scheduling. ECRTS, pp. 6:1–6:23, 2019. https://doi.org/10.4230/LIPIcs.ECRTS.2019.6

[14] からあげ. ルンバをRaspberry Piを使ってROSでコントロールできるように改造してみた（半田付け無しでOK）. 2017-05-12. https://karaage.hatenadiary.jp/entry/2017/05/12/073000

文献[10]はYOLOの原著であり、使用したUltralytics版の構造・性能の直接証拠ではない。[13]はROS 2 Crystalの解析で、本システムの時間保証ではない。[14]はRaspberry PiとROSによる参考実装で、Pico W・ROS 2構成とは異なる。

## 追記 生成AIを用いた実機開発から得た教訓

本開発では、生成AIによる実装支援の一方で、安全性への懸念から実機試験に進むことへ過度に慎重になり、動作を確認するための試験そのものを拒否・先送りする挙動が見られた。また、問題への対処として、開発規模を超える機能追加や構造変更を提案・実施し、検証対象と作業量を増やす傾向があった。

この経験から、生成AIに任せる際は、開発者側が検証の枠組み（ハーネス）を管理する必要があると学んだ。具体的には、開発規模、変更範囲、受入条件、実機試験の条件と実施権限をSKILL.mdなどに明記する。合意した条件が満たされた試験は実施し、新たな懸念がある場合は、具体的な根拠と必要最小限の対応を示す運用とする。

安全対策は実機での検証を前進させるために設計し、対策の追加自体が開発目的にならないよう管理する。
