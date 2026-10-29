# 有人Navigation候補・ARM同期補助 非走行検証

日付: 2026-10-29

## 目的

追加統合試行では、Navigation候補のSTOPからARM受理まで約23.26秒空き、ARM後に
新鮮な非ゼロ候補が届かなかった。人間承認を省略せず、承認後の操作時間だけを
候補受信と同期させる補助を追加した。

## 実装

- `attended_nav_arm_gate.py`は`/nav/cmd_vel_unstamped`を購読する。
- 固定generationと確認文字列`ATTENDED_FLOOR`を起動時に必須とする。
- 0.25秒以内の非ゼロ候補を2回連続受信した場合だけARMを1回publishする。
- `/roomba/arm_generation`のsubscriberが0件なら候補列を破棄して待機する。
- subscriberが2件以上なら重複として停止する。
- timeout時はARM 0件で異常終了する。
- Drive topicのpublisherを持たず、自動再ARMやgeneration自動生成も行わない。

これは人間の当該試行承認後に明示起動する同期補助であり、承認そのものを自動化
しない。既存の20 mm/s上限、400 ms期限、500 ms watchdog、STOP latchは変更しない。

## 非走行検証

Pico、Roomba、USB cameraを未接続とし、Gazebo/Create3と物理Roomba topicも不在の
ROS 2コンテナで実施した。

1. 明示型の模擬ARM subscriberを1件作成。
2. `linear.x=0.08 m/s`のNavigation候補を3件送信。
3. generation `123456789`のARMを**1件だけ**受信。
4. `/roomba/cmd_vel_unstamped`は`Unknown topic`で、Drive出力なし。
5. 候補なし試験ではtimeoutでexit 1、ARMファイル0 byte、Drive topicなし。

初回ハーネスではROS CLIの型推定ができずsubscriber 0件となったが、製品ノードは
ARMせずfail-closedした。明示型rclpy subscriberへ変更後に正常系を確認した。

## 回帰

- 追加単体: 7/7 PASS。
- 全回帰: **380/380 PASS**。
- Python compile、Docker image build、`git diff --check`: PASS。

## 判定

同期補助のコード・mock ROS E2E: **PASS**。
実カメラNavigation候補との非走行E2E: **未確認**。
実Pico ARM/Driveと物理停止: **未確認**。

次は走行ロックFalse、物理bridgeなしで、実カメラ候補を入力して補助ノードの単一ARM
publishを模擬subscriberで観測する。その後に改めて有人実機GO/NO-GOを判定する。

