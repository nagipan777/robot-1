# 無人・自律実機走行 Safety Go / No-Go Checklist

更新日: 2026-10-20。空欄、`FAIL`、`UNKNOWN`が1つでもあれば **NO-GO**。

| 項目 | 状態 | 証拠 |
|---|---|---|
| バッテリー外観・電圧・温度が正常 | UNKNOWN | 当日記録 |
| UART診断3/3、115200 bps | PASS（前回）/当日再確認要 | raw応答ログ |
| STOP ACK 0x07、要求左右0 | PASS（前回）/当日再確認要 | bridge/Picoログ |
| OI mode 2 Safe | PASS（前回）/当日再確認要 | sensor 35 |
| wheel drop / cliff / charging sourceが試験条件と整合 | UNKNOWN | sensorログ |
| protocol v2 CRC・sequence・UART ACKが双方へ配備 | PASS（コード）/配備確認要 | version/hash |
| watchdog 500 msとラッチ拒否 | PASS（503 ms） | Picoログ |
| TCP切断・例外後に再ARM必須 | PASS（mock）/当日確認要 | test/log |
| Pico電源断時の物理停止 | **FAIL・未検証** | なし |
| Pico TX–Roomba RX断線時の物理停止 | **FAIL・未検証** | なし |
| 独立した物理停止・拘束手段 | UNKNOWN | 当日写真・担当者 |
| 試験区域隔離、段差・人・動物なし | UNKNOWN | 当日写真 |
| 有効車輪間距離と床面符号 | UNKNOWN | 実測 |
| 床面速度・時間・距離上限の承認 | UNKNOWN | レビュー記録 |
| `/robot/pose` Publisherが1つだけ | UNKNOWN | `ros2 topic info -v` |
| ArUco寸法・高さ・offset・yaw検証 | UNKNOWN | 校正記録 |
| 4安全フラグの開始前/終了後false | PASS（repo既定）/当日確認要 | param/log |

## OI仕様に基づく境界

Safe modeではcliff、wheel drop、給電中charger条件でRoombaが全モーターを停止しPassiveへ戻る。
Full modeはこれらを無効にするため使用禁止。Safe modeはPico電源断、UART断線、機械故障を検知して
必ず停止させる独立回路ではない。要求速度packet 41/42のゼロも物理停止そのものの証明ではない。

参照: [iRobot Roomba 500 Open Interface Specification](https://cps-vo.org/sites/cps-vo.org/files/cpsvo_file_nodes/iRobot_Roomba_500_Open_Interface_Specpdf.pdf)

## 現在の判定

**NO-GO**。この表は無人／自律走行に適用する。有人限定Stage 2は別紙
`ATTENDED_FLOOR_DRIVE_CHECKLIST.md`で判定し、安全要件を削除せず試験規模に合わせて分離する。
