# 2026-10-31 非走行安全修正

## 適用範囲

対象はGitHubブランチ`fix-arm-ack-session-recovery`の`8b7501e`を基準にした変更である。
Pico W、Roomba、実機ROSネットワーク、非ゼロDriveは使用していない。
**非走行の回帰テストのみで、修正後コードの実機動作は未検証**である。
提出済み実験証拠が参照する元コミットSHAと、そのコミット上の証拠は変更していない。

400 ms drive-session期限、500 ms watchdog、20 mm/s上限、ARM generation、sequence、
CRC、`ALLOW_NONZERO_DRIVE=False`の既定値は変更していない。

## 修正1: 量子化ゼロを正常STOPとして扱う

微小な非ゼロTwistが左右0 mm/sへ量子化され、`send_twist()`がSTOPを送って`None`を
返した場合、Drive ACKを整形しない。commandを消去し、bounded sessionをラッチし、
`idle_stop_sent=True`として接続を維持する。transportによるARM失効は変更しない。

- テスト: `test_bounded_quantized_zero_is_stop_without_reconnect_or_stop_spam`
- 修正前: TypeErrorが送信障害処理へ流れ、`close_calls=1`でFAIL。
- 修正後: close 0、STOP 1回、ARM失効、次tickのSTOP追加送信なし。
- 通常Drive ACK sequence経路も同じテストでPASS。

## 修正2: STOP-first診断でARMを失効

STOPをUARTへ完全書込みできた`DIAGNOSTIC_STOP`とstop-first OI mode診断から、既存の
`MotionLatch.normal_stop()`を呼ぶ。`ARMED`/`ARM_PENDING`は`STOPPED`となる。
`LATCHED_STOP`は解除せず、live OI診断は状態とwatchdogに触れない。

- テスト: `test_diagnostic_stop_revokes_arm_until_newer_generation_rearms`
- テスト: `test_stop_first_oi_diagnostic_revokes_arm_but_live_query_does_not`
- 修正前: 両方とも診断後が`ARMED`のままでFAIL。
- 修正後: 同世代Drive拒否、より大きいgenerationの再ARM後だけDrive許可。
- `test_stop_and_diagnostic_do_not_release_latched_stop`: PASS。
- `test_deployed_and_repository_relay_match_except_deployment_comment`: PASS。

## 修正3: UART待機中も既存期限を評価

`drain_uart_until_quiet()`、`read_uart_exact()`、`query_uart()`の待機中に、既存の
drive-session期限、command watchdog、ARM watchdogを評価する。最終Drive時刻は
`MotionLatch`内部だけに保持し、プロトコルには追加しない。期限成立時は照会を続けず
空応答を返す。STOP書込みと`latch_stop()`は既存期限関数が行う。

- テスト: `test_live_uart_wait_is_interrupted_by_drive_session_deadline`
- 修正前: 無応答照会後も`ARMED`のままでFAIL。
- 修正後: FakeTimeの400 ms境界で中断、STOP書込み、`LATCHED_STOP`。
- `test_idle_uart_wait_keeps_existing_diagnostic_timeout`: 非駆動時の従来timeoutを維持。
- `test_live_uart_wait_with_complete_response_keeps_arm_and_payload`: 正常応答を維持。

## 回帰結果

- 正規コマンド: `env ALLOW_NONZERO_DRIVE=False drive_enabled=false control_enabled=false calibration_ready=false .venv/bin/python -m unittest -v`
- 結果: **380/380 PASS**。
- 全てモック、FakeTime、ローカルloopbackによる非走行試験。実機は未検証。

## 未対応・理由

- host `arm_hold_timeout=1.0 s`とPico ARM窓500 msの差: ARM直前Safe確認と初回Drive経路を含む設計変更になり、実機確認なしでは変更しない。
- Drive拒否ACK `0x01`と通信障害の区別: host状態機械と運用判定を変えるため今回は対象外。
- Pico接続アイドル回収とSTOP keepalive: TCP session運用の別課題として分離する。
- 例外時の`machine.reset()`: 自動復帰を追加しないという制約を維持する。
