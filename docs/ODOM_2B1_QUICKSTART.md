# Phase 2B-1：実験者用クイックシート

**対象ブランチ：`tooling/simple-odometry-operator-20261103`。Macのリポジトリルートで実行。**

本手順は、既存の「静止encoder取得」と「走行前後差分計算」の操作をまとめたものです。**走行そのものは自動化しません。** ARM、Drive、Navigation、TCP、UART、STOP、watchdogの実装は変更していません。

## 操作は4段階

0. **事前に**Roomba/Picoのプログラム版・無通電時の設定・STOP/Safe診断と他ブリッジの不存在を既存手順で確認し、対象試行の個別承認を得る。Picoを手動起床し、実験者が動きを監視する。コンテナは起動済みであること。

1. **走行前**：`./tools/odom2b1.sh before PICO_IP`
   - STOP-first encoder10件を保存。失敗したら走行せず終了。
2. **有人試験**：`./tools/odom2b1.sh approve` で承認を記録後、**既存の検証済み試験ランチャーを別途使用**。このスクリプトはDriveを発行しない。試行終了後、両輪の物理停止と再発進なしを確認。
3. **走行後**：`./tools/odom2b1.sh after PICO_IP MEASURED_MM`
   - 例：`MEASURED_MM=0`は実測が本当に0mmの場合のみ。
   - `PHYSICAL_STOP_CONFIRMED`の入力後、STOP-firstでencoderを取り、距離・yaw差分をCSV保存。
4. **結果**：`./tools/odom2b1.sh status` で`result.csv`の場所を確認。次回は`./tools/odom2b1.sh new-session`でポインタだけをクリア。

**手順の制約**：before→afterの間にRoombaを電源再投入しない。走行中のencoder取得を行わない。別のTCP所有者がある場合は開始しない。通信エラー時に自動再ARM・Drive再送はしない。

## 重要：操作が簡単でも、測定の有効性は別

- 走行後の再接続まで長時間空いた場合、encoder差分をDrive区間に帰属させない。
- 静止時にもセッション間で約2〜4tickの差が発生した記録がある。数tickの差を走行証拠にしない。
- 既存の400ms・20mm/s試験では実移動0mmだったため、「通信PASS」と「実移動PASS」を分ける。
- このラッパーは**ファイル操作・既存静止診断の操作窓口**であって、live ARM/bridge/STOP/車輪状態を監視する完全なGUIではない。
- `approve`はログ上の承認記録であり実機へARMを送らない。試験の実施判断・物理停止確認は実験者が行う。

## 何が改善されたか

長い`run_encoder_stop_interval.py prepare/authorize/finalize`のオプション指定を隠して、試験時に必要な入力をPico IPと実測距離に絞りました。既存のCSV・JSON・session evidenceは削除せず保存します。

**未対応**：試験ランチャーの統合、全ブリッジの統一start/status/stop、実機の自走が0mmの原因究明。これらは本変更で実現したと主張しません。
