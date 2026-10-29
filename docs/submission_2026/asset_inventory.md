# 提出素材インベントリ

監査日: 2026-10-29。リポジトリ内のみを調査し、外部Mac写真ライブラリ等は含めない。

## 動画

| 素材 | 区分 | 状態 |
|---|---|---|
| `artifacts/submission/pai_create3_20261015T163837JST/pai_create3_20261015T163837JST_timelapse_10x.mp4` | Create3/Gazebo simulation | 利用可。唯一検出した動画 |
| Roomba 10/28 AI床面試験動画 | 実機 | **未収録/リポジトリ内に実ファイルなし** |
| Roomba 10/29 P3-2浮上試験動画 | 実機 | **未収録/リポジトリ内に実ファイルなし** |
| Roomba 10/29追加AI統合FAIL | 実機・Drive 0件 | **未収録/リポジトリ内に実ファイルなし** |
| ARM同期補助の実カメラdry-run動画 | real camera / non-driving | **未収録/ログと報告書のみ** |
| 10/29 単一ブリッジAI連動単発 | 実カメラ・Roomba実機 | 画面収録済み。1:54–1:56付近に微小移動。7.033秒抜粋を`artifacts/submission_2026/video_clips/unified_ai_roomba_motion_20261029.mp4`へ作成 |
| 10/29 mask/ArUco非走行とSTOP診断 | real camera / terminal | **ユーザー端末へ撮影済み。リポジトリ内パス/hash未登録** |

静止画やログから実機映像を生成・補間しない。10月29日の撮影ファイルは、提出素材へ
コピーして実ファイル、内容、hashを確認するまでは「利用可能」と確定しない。単一
ブリッジ試験のYOLO画面収録だけは実体とhashを確認済みである。スマートフォン側は
写真のみで、実機外観動画は収録できていない。

## 画像

- YOLO/Create3: `artifacts/submission/*/yolo_detection.jpg`、`artifacts/submission/*/frames/`
- camera calibration: `artifacts/camera_calibration_20261028/`（内部較正、map点、yaw zero）
- YOLO評価可視化: `artifacts/pai_output_validation/fixed_confidence/visualizations/`
- 監査時の概数: PNG 54、JPG 999（重複frameを含む）

## ログ・構造化証拠

- AI床面: `artifacts/step41/ai_attended_floor_trial_20261028.json`
- deadline解析: `artifacts/ai_attended_floor_20261029/deadline_analysis.json`、`deadline_retest_100hz.json`
- ARM/TCP: `artifacts/pico_ping/arm_ack_lock_probe_20261027*.json`
- Navigation: `artifacts/navigation_latency_20261025/*-navigation.log`、`*-summary.json`
- 実カメラE2E: `artifacts/e2e_real_camera_pico_dry_run/`
- odometry: `artifacts/odometry_dryrun/e2e/ros_odometry.csv`、`offline/comparison.csv`、`offline/trajectory.svg`
- wheel monitor: `artifacts/wheel_motion_monitor/dryrun/`（CSV/summary）
- 監査時の概数: CSV 30、JSON 95、LOG 45

## 重要hash

- v2 model: `3e1de3c72c6d807fd527cefc6b70801275e3a9b54bb9daa8d87a59dc7be12b03`
- AI床面JSON: `509868c9f1c1e238769f344f137aa649d52793d550d9a9358094742532d4818d`
- Create3動画: `7b6fa713a7e34920fa6495e3afa052b332e2802aab3400659ba4496d66229802`
- odometry/wheel-monitorのartifact hashは各報告書に記録済み

## 追加撮影が必要な最小カット

1. 実カメラでmask bboxとArUco ID 0の同時表示（非走行）の画面収録。
2. STOP ACK `0x07`、左右0、Safe 2のterminal画面。
3. 既存成果を再現する場合のみ、人間承認付き20 mm/s/400 ms単発と、その直後5秒の停止。再試験しない方針なら「実機映像なし」と明示し、ログを画面表示する。
4. 最終lock FalseとPico無通電。

最小構成では1、2、4を安全な非走行撮影で補い、実機走行映像だけを不足カットとして
残せる。実機走行の再撮影は提出資料作成の必須条件ではなく、安全ゲートと個別承認を
満たした場合だけ行う。

公式提出形式・動画時間の一次資料はリポジトリ内で確認できなかった。3–5分構成は提案であり公式要件ではない。

