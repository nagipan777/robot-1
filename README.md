# robot-1 — AI認識とRoomba 680実機Navigation

**YOLOでごみを見つけ、ArUcoでRoombaの位置・向きを推定し、ROS 2で移動指令を計算して、Pico W経由でRoomba 680を動かすプロジェクト。**

**[最終課題の趣旨に沿った成果要約（タスク・AI・実機動作・動画・課題）](docs/submission_2026/task_focused_summary.md)** を最初に確認してください。講義課題向けの要点をまとめています。

PAI応用講座の最終課題として開発し、2026年10月に動画を提出しました。現在は追加開発として、連続直進・左右旋回・実機オドメトリ・Wi-Fi/TCP遅延への対応を進めています。旧READMEはシミュレーター中心の長いチュートリアルでした。このREADMEでは、**完成済み／実験で確認済み／未達／次の予定**を分け、実験者が現在地を把握できることを優先します。

> **最終整理日：2026-10-30。** GitHubで直接確認できる基盤は主に\`fix-arm-ack-session-recovery\`ブランチです。10月30日のC3追加試験は、利用者が共有したローカル実験報告書に基づいて追補しました。最新のC3作業ブランチ・生ログはこのGitHub基準ブランチ上では未確認のため、**報告された実測とGitHub上で再現可能な実装を同一視しません。**

## 目次

1. [目標](#1-目標) / 2. [現在の達成状況](#2-現在の達成状況) / 3. [全体構成](#3-全体構成)
4. [AI推論と制御](#4-ai推論と制御) / 5. [画像認識・カメラ較正](#5-画像認識カメラ較正)
6. [実機統合の到達点](#6-実機統合の到達点) / 7. [通信・安全制御](#7-通信安全制御)
8. [C3連続走行](#8-c3連続走行) / 9. [オドメトリ](#9-オドメトリ)
10. [環境と再現手順](#10-環境と再現手順) / 11. [運用上の課題](#11-運用上の課題)
12. [実装予定・優先順位](#12-実装予定優先順位) / 13. [資料・ファイル一覧](#13-資料ファイル一覧)
14. [証拠の扱い](#14-証拠の扱い)

---

## 1. 目標

目指しているのは、床面の対象を検出してから、その方向へ旋回・前進し、姿勢を更新しながら近づき、到達条件で停止する一連の**閉ループNavigation**です。

\`\`\`text
カメラで検出 → ごみ位置を推定 → Roombaの姿勢を知る
      → 進行方向を計算 → 左右車輪を動かす
      → 姿勢と目標距離を更新 → 停止
\`\`\`

すでにAI認識から実機の左右車輪を動かすところまでは結合できました。ただし、**ゴミへの連続接近・自律到達・吸引完了**は実機で未達です。無人運転にも進んでいません。RoombaのOpen Interface（OI）を用いる研究・学習用の実装であり、市販ロボット掃除機としての安全保証を主張するものではありません。

最終課題提出時の範囲と、提出後の追加実験は区別します。シミュレーションPASSは実機PASSではなく、通信成功は実距離の証明でもありません。

## 2. 現在の達成状況

| 機能 | 状態 | 検証内容 |
|---|---|---|
| Create 3／Gazebo Navigation | **シミュレーションPASS** | 旋回、接近、到達、停止 |
| YOLOでの実画像検出 | **限定PASS** | 固定環境・前方60〜80cm |
| ArUco ID 0の実カメラ位置推定 | **較正／非走行PASS** | map座標の位置・yaw候補 |
| YOLO→ROS 2 Navigation | **非走行PASS** | Twist生成、検出途絶STOP |
| AI→Pico→Roomba | **有人単発PASS** | 10/28、左右20/20mm/s、STOP |
| ARM ACK消失への修正 | **限定実機PASS** | ARM送信ロジックの修正、Pico試験 |
| host deadline対策 | **一部実機PASS** | ACK待ち中断→STOP。ただし厳密STOP開始時刻未保存 |
| C3：1秒超の同一ARM継続Drive | **10/30報告上PASS** | ACK seq 1〜22 |
| C3：直進距離 | **距離条件PASS** | 実測56mm、許容40〜80mm |
| C3：規定時間の通信連続性 | **FAIL** | seq 23はwatchdog STOP後に到着 |
| C3：ACK再同期 | **報告上PASS** | STOP後の実行済みsequence照合 |
| Navigation左右旋回・目標到達 | **未完了** | 実機で旋回角・距離・再現性が必要 |
| \`/roomba/odom\` | **dry-run E2E PASS** | CSV encoder→ROS 2 Odometry |
| 実機encoder 43/44 | **未統合** | Pico→ROS 2データ経路が未完成 |
| 長時間・無人自律走行 | **未検証** | 有人限定試験とは別 |

C3の実走行56mmは重要な成果ですが、それだけでC3の連続通信条件までPASSにはしません。同様に、停止が成功したことを、厳密な停止保証の一般化には使いません。

## 3. 全体構成

Macの\`vision.py\`がカメラ画像を読み、Colima/Docker内のROS 2が目標・姿勢・速度を計算し、Pico Wが唯一のUART所有者としてRoombaへDrive Directを送ります。

\`\`\`text
[固定USBカメラ 800×600]
          │
          ▼
[Mac：vision.py]
   YOLO物体検出 ＋ OpenCV ArUco検出
          │ HTTP検出JSON
          ▼
[Colima / Docker：ROS 2 Jazzy]
   webcam_bridge → /webcam/detections
         ├─ 床面目標 → /litter/target
         └─ ArUco姿勢 → /robot/pose
                       │
             vision_turn_controller
                       │ /nav/cmd_vel_unstamped
                       ▼
             cmd_vel_output_adapter
                       │ /roomba/cmd_vel_unstamped
                       ▼
                roomba_bridge_tcp
             ARM・STOP・ACK・期限
                       │ TCP/Wi-Fi
                       ▼
                 [Pico W]
           単一UART／走行許可・監視
                       │ Roomba OI
                       ▼
                 [Roomba 680]
                   左右車輪
\`\`\`

シミュレーションでは、出力先をCreate 3/Gazeboへ切り替えます。実機／Gazebo／dry-runを混同せず、実機出力とシミュレーターの速度指令が同時に有効にならない構成を維持します。

| 主要トピック | 用途 |
|---|---|
| \`/webcam/detections\` | 検出JSON、画像上のbbox・confidence |
| \`/litter/target\` | map座標のごみ目標 |
| \`/robot/pose\` | map座標のRoomba姿勢 |
| \`/nav/cmd_vel_unstamped\` | NavigationのTwist候補 |
| \`/roomba/cmd_vel_dry_run\` | 非走行の出力確認 |
| \`/roomba/cmd_vel_unstamped\` | 実機ブリッジ入力 |
| \`/roomba/arm_generation\` | 明示的ARM要求 |
| \`/roomba/encoder_counts\` | encoder入力（現在は主に模擬） |
| \`/roomba/odom\` | 差動二輪Odometry（実値未接続） |

詳細は[ARCHITECTURE.md](ARCHITECTURE.md)、[LOCALIZATION_ARCHITECTURE.md](LOCALIZATION_ARCHITECTURE.md)を参照してください。

## 4. AI推論と制御

**今回の学習済みAI推論は、Mac上のYOLOが担当します。** YOLOはカメラ画像から「対象がどこにあるか」をbboxと信頼度で返します。ArUcoはOpenCVによるマーカー検出と幾何変換、Navigationは座標差・角度差に基づく速度計算です。Pico WもAI推論は行わず、Drive/STOPの通信と走行許可を扱います。

- **YOLO**：床の対象を検出する画像認識モデル。
- **ArUco**：Roomba上のID 0を認識し、固定カメラから姿勢を推定。
- **Navigation**：目標と現在姿勢から、前進するか旋回するかを計算。
- **Pico W**：ARM状態、connection ID、Drive sequence、UART書込みとSTOPを管理。

SmolVLA等のVLAが直接ロボット行動を推論する方式とは違い、ここでは**AI物体認識＋古典的なロボット制御**を統合しています。

\`vision_turn_controller.py\`には\`turn_only\`と\`approach\`があり、目標方向から大きくずれていれば旋回し、向きが合えば低速前進する設計です。ただし、実機の20mm/s上限ではシミュレーターより旋回速度が制約されるため、実旋回角の定量評価と制御ゲインの調整は今後の課題です。

## 5. 画像認識・カメラ較正

### YOLOの学習と運用条件

TACOから学習を開始し、Gazebo画像や実環境の負例を加えました。Roomba本体、黄色いテープ、床面の背景を誤ってゴミと認識する問題があり、hard-negativeで対処しています。

2026-10-28の実カメラ実験では、位置違いに対応したv2候補を使用しました。

\`\`\`text
weights: runs/detect/real_finetune/weights/best_position_v2_20261028.pt
SHA256: 3e1de3c72c6d807fd527cefc6b70801275e3a9b54bb9daa8d87a59dc7be12b03
camera: 800×600 / confidence: 0.50 / imgsz: 960
\`\`\`

確認できたのは固定環境の前方60〜80cm程度での検出です。100cm付近では途切れがあり、未知のごみや一般環境の高精度検出までは主張しません。モデル重みはGitHubに存在しない場合があるため、ローカルでパスとSHA256を照合してください。

### ArUcoと床面投影

使用したのは\`DICT_4X4_50\`のID 0、黒枠外辺80mm。実験室の固定カメラ高さ0.53m、マーカー面高さ0.12m。チェッカーボード7画像の内部較正RMSは0.7438px、床面6点の当てはめRMSEは0.0193m、独立検証2点の誤差は0.0427mと0.0243mでした。

これは**固定カメラ・固定配置における座標投影の検証**で、どこでも同じ位置精度が得られるという保証ではありません。カメラ位置や画角を変えれば再較正が必要です。詳しくは[カメラ較正記録](docs/CAMERA_CALIBRATION_20261028.md)を参照してください。

### 非走行で確認したこと

実カメラからROS 2へ検出を送り、Navigation候補を生成。マスクを取り除くとSTOPし、ArUcoが見えなくなった場合も非ゼロ出力を止めます。同じ目標を再表示しても自動的には再発進しません。この\`SafeTargetManager\`の保守的な挙動は、短時間の有人試験には合いますが、継続的なごみ接近では再認識と再開認可を分離して検討する必要があります。

## 6. 実機統合の到達点

### 10月28日：AI連動実機

実カメラのYOLOとArUcoからNavigationの前進指令を出し、明示ARM後にRoombaへ左右20/20mm/sを送信しました。PicoではDrive sequence 1〜7を確認。最初のDriveから通常STOPまで289ms。立会者は短時間前進と物理停止、異音なし、再発進なしを確認しました。

これが**「AI連動実機統合PASS（限定有人単発）」**の根拠です。一連の自律接近・ゴミ回収に成功したという意味ではありません。一次証拠は[10月28日試験JSON](artifacts/step41/ai_attended_floor_trial_20261028.json)です。

### 10月29日：deadline障害と修正

旧20Hz監視ではホストSTOPが約41ms遅延。100Hz timerへ変更しても、同期Safe照会やDrive ACK待ちがsingle-threaded executorを約181ms塞ぐ問題がありました。TCP STOPの送信処理自体は主因ではありませんでした。

同一TCP所有者が応答待ち途中で絶対期限を監視し、期限到達時にSTOP・ARM失効・接続破棄する方式に変更しました。有人両輪浮上試験でPicoのDrive→STOP 224ms、STOP後Drive 0件、物理停止と再発進なしを確認。

ただし、その試行ではホスト側STOP送信開始の厳密なtraceを保存していません。後続エラーログは開始から400.734msでしたが、それをSTOP開始時刻とはみなしません。ログ改善コードは回帰PASS、改善後の追加実機走行は未実施です。[期限修正報告](P3_2_DEADLINE_IO_SEPARATION_REPORT_20261029.md)。

## 7. 通信・安全制御

### RoombaとPico

Pico Wを介してRoomba OIへUARTで接続します。現行経路はGP4 TX・GP5 RXと共通GND、電圧レベル変換器を使用。BRC自動起床は使わず、CLEAN手動起床を採用しています。初期のUSB-UART直接接続や19200bpsの実験を、現在のPico W/TCP運用と混同しません。

### ARMとDrive

走行前にARM generationを明示的に承認し、PicoはTCP connection IDとDrive sequenceも確認します。STOP後に古いARMや未確定Driveを使い回す設計ではありません。非ゼロDriveには別途許可が必要で、待機時の\`ALLOW_NONZERO_DRIVE=False\`を基本とします。

提出時点の短時間モードは20mm/s、ホスト/Picoの400msセッション上限、更新途絶500ms watchdog、STOPラッチを持ちます。**C3は別の限定連続走行モード**であり、通常モードの制限を黙って延長したものではありません。

### ARM ACKの障害

旧Picoでは、MicroPythonの\`socket.send()\`がノンブロッキング動作で\`None\`を返した際、残りバイトの送信が完了したと誤認していました。ACK 0/7byte障害の再現後、全バイト送信、\`ARM_PENDING\`、ACK送信完了後commit、接続ID拘束へ修正しました。修正版の実Picoロック診断は10/10 PASSです。

### STOPと観測

STOP ACK \`0x07\`はPicoによるSTOP要求検証・UART書込み・Roomba要求車輪速度0を示します。ただし、物理的に車輪が止まったかは別の確認です。Wi-Fiが切れた場合に最後の速度指令を保持し続けないよう、Picoの独立停止監視を用います。一方で**Pico自体が停止・無通電になった際の物理停止を、その監視だけで保証することはできません**。

通信の信頼性と停止機構の有効性は分けて判定します。

## 8. C3連続走行

> ここは2026-10-30に提供されたローカル報告書の追補です。最新のC3コードとログがGitHubに反映されたかは未確認です。該当コミットが確認できた後に実行コマンドとリンクを追加します。

C3の目標は400ms単発を越え、同一ARMで1秒超のDriveを連続送信し、60mm前後の直進を計測することです。試験条件は左右20/20mm/s、指令2.8秒、固定期限3.0秒、通信watchdog 500ms、Drive再送なし、自動再ARMなし。距離の暫定許容帯は40〜80mm。

| 試験 | 1回目 | 2回目 |
|---|---:|---:|
| 正常Drive ACK | seq 1〜2 | seq 1〜22 |
| 次のDrive | seq 3拒否 | seq 23拒否 |
| 距離実測 | 26mm | **56mm** |
| 距離40〜80mm | FAIL | **PASS** |
| 同一ARMで1秒超 | 未達 | **PASS** |
| 全指令区間の通信継続 | FAIL | **FAIL** |
| 物理停止・再発進なし | PASS | PASS |

1回目は走行中の同期Safe照会がDrive更新を阻害。限定連続試験では、走行前に確認したSafe結果を使い、走行区間の同期ライブ照会を分離しました。その結果、2回目はACK 22件を確定し、56mm進みました。

しかしseq 23は最終Driveから約619ms後にPicoへ到着し、500ms watchdogですでにSTOPラッチされていたため拒否されました。seq 22のACKをホストが待った時間は310.911msで、ACK確定からseq 23送信まで4.539ms、ホスト\`sendall\`は0.174ms。遅延はTCP/Wi-Fi経路のどこかにありますが、同一床面試験のパケットキャプチャがなく、AP・CYW43/lwIP・macOSのどこかは断定できません。

また別試験で、未確定ACKについてSTOP後にboot ID・generation・sequenceを台帳照合する機能を実機検証したとの報告があります。**これは実行結果の再同期であり、通信遅延中に走行を続けられる能力ではありません。**

C3の結論は、**移動距離と一度のARMでの1秒超駆動はPASS、予定した通信連続性はFAIL**です。どちらも記録して次の改善に使います。

## 9. オドメトリ

\`odometry.py\`、\`simulation/odometry_dryrun_core.py\`、\`simulation/wheel_odometry_node.py\`には差動二輪オドメトリの計算があり、左右エンコーダ値からx・y・yaw・速度を積分します。16bit wraparound、入力欠損、異常ジャンプ、通信断後の再基準化を扱います。

\`\`\`text
CSV replay（模擬）
 → /roomba/encoder_counts
 → wheel_odometry_node
 → /roomba/odom（nav_msgs/Odometry）
 → CSV保存・RViz・ArUco模擬比較
\`\`\`

\`./simulation/sim.sh odometry-dryrun-e2e\`で、ROS 2ノード間のdry-run E2Eを確認しています。入力6件のうち有効Odometry5件で、timeout入力は姿勢へ積分しません。模擬ArUco比較がほぼ誤差0でも、実機位置精度を示す数値ではありません。

**未完成なのは実機データの入口です。** OI packet 43・44の左右encoder値をPico Wで取得し、単一UART所有者を保ったままTCP→ROS 2の\`/roomba/encoder_counts\`へ送る経路は未統合。STOP監視をセンサー照会でブロックしないよう、最初は静止時の読み取りから検証します。Navigationの姿勢入力は当面ArUcoのままとします。

## 10. 環境と再現手順

### 利用環境

Apple Silicon Mac、Python 3、Docker、Colima、ROS 2 Jazzy、Gazebo Harmonic、Create 3。実カメラ用のUSBカメラとYOLO重みが必要です。実機試験にはPico W、レベル変換器、Roomba 680を使用します。

\`\`\`bash
git clone https://github.com/nagipan777/robot-1.git
cd robot-1
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
git rev-parse HEAD
\`\`\`

\`main\`は最新の実機開発内容を含まない可能性があります。再現時には使用したブランチとcommit SHA、モデルファイルSHAを明示してください。

### シミュレーション（Macから）

\`\`\`bash
./simulation/sim.sh up
./simulation/sim.sh status
\`\`\`

VNC：\`http://localhost:6080/vnc.html?autoconnect=true&resize=scale\`。動画撮影やGazeboの詳細は[simulation/README.md](simulation/README.md)にまとめています。

### オドメトリdry-run（実機接続なし）

\`\`\`bash
./simulation/sim.sh odometry-dryrun-e2e
\`\`\`

結果は\`artifacts/odometry_dryrun/e2e/\`へ保存されます。これはPicoへのARM/Driveを送らない試験です。

### ROS 2の状態確認

\`\`\`bash
./simulation/sim.sh ros-status
\`\`\`

ROS環境が起動していることが前提です。トピックが見えることだけでは、Roombaへの出力やARM成功を意味しません。実機コマンドは各試験のrunbookに従い、READMEにある断片的な例を直接つないで運転しません。

## 11. 運用上の課題

### 実験の主導権

実験者が目標・合否・実施タイミングを決め、Codex等は実装・回帰・ログ整理を支援する方針です。たとえば「60mm直進」を目的にしたとき、ACK再同期の完成をC3全体のゴールに置き換えず、補助成果として記録して元の目的へ戻ります。

### 孤児ブリッジ

過去には中断された\`docker compose exec\`の後にSTOP専用ブリッジが残り、ARM購読者が重複しました。有人ブリッジの起動経路にはPID、\`flock\`、heartbeatがありますが、**診断を含むすべての起動方法が統一管理されているわけではありません**。

将来はMacターミナルから一意の\`start/status/stop\`相当で操作し、リビルド前に残存ブリッジの有無を確認できる仕組みへ整理します。既存の起動時STOP/Safe診断を活用し、別ブリッジを何度も起動し直さない運用が目標です。

### 手順書の短縮

現場向けはA4一枚・最大5操作程度のクイックシートを基本とします。「Mac／コンテナ／Pico」の実行場所、実在するコマンド、期待出力、停止方法を明記。詳細な障害解析・状態遷移・回帰は技術資料へ分けます。手順書のタイポや実在しない引数は実コードとの照合で減らします。

## 12. 実装予定・優先順位

| 優先 | 課題 | 次に確認すること |
|---|---|---|
| **P0** | C3通信の途絶 | 同一試行でmacOSパケット捕捉とPico/hostログを照合 |
| **P0** | 左右旋回の実機Navigation | 左右輪符号、実際の旋回方向、ArUco yaw差 |
| **P0** | ブリッジのライフサイクル | start/status/stop統一、孤児・多重起動防止 |
| **P1** | 60mm前後の直進再現性 | 距離・指令時間・ACK継続性を独立評価 |
| **P1** | 実機encoder 43/44 | 静止取得→Pico→TCP→ROS 2接続 |
| **P1** | ArUco／オドメトリ比較 | 同一試験で距離・方位・時刻を比較 |
| **P2** | YOLO検出途絶の復帰 | 停止維持と再認識・再許可を分離 |
| **P2** | 画像の鮮度 | 撮影時刻・推論時間・通信遅延を区別 |
| **P3** | 検出の一般化性能 | 未知背景・距離・照明・独立holdout |
| **P3** | 長時間の無人走行・回収 | 別の仕様と実機検証が必要 |

**直近ロードマップ**

1. 実験ブリッジをMacから確実に操作できるようにし、古いプロセスの残存を防ぐ。
2. 直進と左右旋回を独立に測定。指令の符号だけでなく物理的な方向・角度・距離を記録。
3. packet 43/44を実機で取得し、ROS 2の既存Odometryノードへ流す。
4. wheel odometryとArUco姿勢を比較し、滑り・ずれ・時刻誤差を評価する。
5. ゴミを目標に姿勢更新しながら接近する閉ループを作る。

「まず動かす → 記録する → 問題を見つける → 最小限直す → 再試験する」という実機開発の反復を重視します。安全機構の追加それ自体を目的化しません。

## 13. 資料・ファイル一覧

| 資料 | 主な内容 |
|---|---|
| [ARCHITECTURE.md](ARCHITECTURE.md) | システム・ROSトピック構成 |
| [LOCALIZATION_ARCHITECTURE.md](LOCALIZATION_ARCHITECTURE.md) | 座標系・位置推定 |
| [docs/CAMERA_CALIBRATION_20261028.md](docs/CAMERA_CALIBRATION_20261028.md) | ArUco・床面較正 |
| [ARM_TCP_RECOVERY_20261026.md](ARM_TCP_RECOVERY_20261026.md) | ARM ACK 0/7問題 |
| [P3_2_DEADLINE_IO_SEPARATION_REPORT_20261029.md](P3_2_DEADLINE_IO_SEPARATION_REPORT_20261029.md) | 期限STOPと同期I/O |
| [AI_ATTENDED_FLOOR_DEADLINE_REPORT_20261029.md](AI_ATTENDED_FLOOR_DEADLINE_REPORT_20261029.md) | AI連動有人実機結果 |
| [ODOMETRY_DRYRUN_REPORT_20261027.md](ODOMETRY_DRYRUN_REPORT_20261027.md) | ROS 2模擬Odometry |
| [WHEEL_MOTION_MONITOR_REPORT_20261029.md](WHEEL_MOTION_MONITOR_REPORT_20261029.md) | 観測用車輪監視 |
| [docs/submission_2026/evidence_matrix.md](docs/submission_2026/evidence_matrix.md) | 実機／模擬／未検証の証拠 |
| [docs/submission_2026/final_report_draft.md](docs/submission_2026/final_report_draft.md) | 提出レポート初稿 |
| [simulation/README.md](simulation/README.md) | Create 3・Gazeboの操作 |
| [pico/README.md](pico/README.md) | Pico側の説明 |
| [PAI_SUBMISSION.md](PAI_SUBMISSION.md) | 旧版の撮影チュートリアル |

C3の最新詳細はユーザー提供のローカル\`C3_CONTINUOUS_FLOOR_RESULTS_20261030.md\`、\`C3_TCP_ACK_DELIVERY_INVESTIGATION_20261030.md\`、\`C3_REAL_ACK_RECONCILIATION_RESULT_20261030.md\`。GitHubへ反映されたら相対リンクを更新してください。

主要コード：\`vision.py\`、\`simulation/vision_turn_controller.py\`、\`simulation/cmd_vel_output_adapter.py\`、\`simulation/roomba_bridge_tcp.py\`、\`simulation/roomba_tcp_transport.py\`、\`pico/main.py\`、\`odometry.py\`、\`simulation/wheel_odometry_node.py\`、\`simulation/sim.sh\`。

## 14. 証拠の扱い

実験結果は必要に応じ\`MOTION\`、\`DISTANCE\`、\`ARM-CONTINUITY\`、\`TRANSPORT-CONTINUITY\`、\`FAILSAFE\`を独立に判定します。未保存・未実施の測定は\`UNKNOWN\`です。

\`Drive ACK 0x03\`はUART書込みの成立、\`STOP ACK 0x07\`はSTOP要求検証・UART書込み・要求速度0を示します。ただし物理停止・床面移動量をそれだけで確定できません。ホストとPicoは別の単調時計を使うため、同じ時計内の差分で評価し、絶対値を直接差し引きません。

56mm進んだ事実と、通信継続が失敗した事実をどちらも残します。試験の一部がFAILでも、実測で成立した部分を消さず、失敗もなかったことにしません。**実測に基づいて次の実験へ進むこと**が、このプロジェクトの開発方針です。

---

**現在地：** AI認識からの限定有人実機駆動は達成。C3では同一ARMの1秒超駆動と56mm直進を報告しましたが、通信の連続性、左右旋回の実測、実オドメトリは未完成です。次は「ブリッジ操作の簡素化」「旋回・距離の計測」「encoderのROS 2統合」を進めます。
