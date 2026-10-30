# 連続直進試験 CSVスキーマ

1行を1試行とし、Drive ACK詳細は同じ`trial_id`の別CSVに1行1sequenceで保存する。

## `trial_summary.csv`

```csv
trial_id,started_at_utc,git_commit,pico_sha256,mode,approval_text,arm_generation,tcp_connection_id,speed_left_mm_s,speed_right_mm_s,host_deadline_s,pico_deadline_ms,watchdog_ms,theoretical_distance_mm,aruco_start_x_m,aruco_start_y_m,aruco_start_yaw_rad,aruco_end_x_m,aruco_end_y_m,aruco_end_yaw_rad,measured_forward_mm,measured_lateral_mm,measured_yaw_rad,aruco_valid,drive_ack_count,first_sequence,last_sequence,max_drive_gap_ms,p95_drive_gap_ms,max_ack_latency_ms,p95_ack_latency_ms,host_stop_begin_s,pico_stop_reason,pico_uart_stop_ms,stop_ack_status,physical_stop,refired_within_5s,result,abort_reason
```

未測定値は空欄にする。ArUco無効をゼロ変位として記録しない。時刻は各機器の絶対時計を
直接減算せず、それぞれの試行開始からの相対時間も保存する。

## `drive_ack.csv`

```csv
trial_id,host_monotonic_s,pico_at_ms,generation,connection_id,sequence,requested_left_mm_s,requested_right_mm_s,ack_status,ack_received_bytes,ack_latency_ms,command_age_ms,pose_age_ms,target_age_ms,fixed_deadline_remaining_ms
```

ACK未確定行は`ack_status`を空欄にし、受信byte数を保存する。同じsequenceの再送行を
新規成功として記録してはならない。
