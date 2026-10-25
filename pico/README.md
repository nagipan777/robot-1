# Pico W / Roomba UART probe

## File roles

- `main.py`: the existing manual UART probe; retained without replacing it.
- `roomba_uart_probe.py`: Drive-free battery/OI/wheel-drop diagnostic probe.
- `roomba_diagnostic_codec.py`: pure decoder for its fixed Query List response.
- `pico_relay.py`: canonical Wi-Fi TCP-to-UART relay source. For deployment,
  save a local copy of this file on the Pico W as `main.py`.
- `cmd_vel_dry_run_receiver.py`: port 9001 log-only velocity receiver. It has
  no UART import or Roomba OI path; deploy it with `dry_run_protocol.py` while
  validating ROS 2 to Pico W communication.

## ROS 2 to Pico W log-only dry-run

Copy `dry_run_protocol.py` and a credential-filled local copy of
`cmd_vel_dry_run_receiver.py` to the Pico W. Run the receiver directly; do not
replace or start `pico_relay.py`. Expected startup output includes
`UART=DISABLED` and port `9001`.

On the ROS side, keep the output adapter in `dry_run` mode, then start the
dedicated bridge:

```bash
./simulation/sim.sh cmd-adapter dry_run /cmd_vel_unstamped
./simulation/sim.sh pico-cmd-dry-run PICO_IP 9001
```

Protocol v2 assigns a new session ID for every accepted TCP connection. READY,
CMD, and ACK frames are protected by CRC-16/CCITT; command sequences restart
within each new session. A client must send its first valid frame within 500 ms,
so an abandoned socket cannot indefinitely block a later connection. ACTIVE
commands retain the 500 ms receive watchdog. An already-STOPPED connection has
a separate 2 second idle timeout and receives 100 ms STOP keepalives, avoiding
false reconnects during ordinary scheduler jitter while still reclaiming stale
sockets. Invalid frames, session/sequence mismatches, timeouts, and TCP
disconnect all transition the Pico logger to `STOPPED` and close the connection.
This state is only a logged software state: this receiver cannot write a Roomba
STOP or drive command.

After reconnect, the ROS bridge always sends STOP first. It rejects non-zero
ROS commands until it observes a new zero ROS command, then accepts only newer
commands from the new session. Failed connection attempts use a bounded 1, 2,
4, then 8 second retry backoff.

Set Wi-Fi credentials only in the deployed local copy. Do not commit SSIDs or
passwords to this repository. The relay accepts only complete, in-range OI
Drive Direct frames and does not extend its 500 ms watchdog for arbitrary TCP
data.

## Safe relay synchronization

1. In Thonny, open the Pico W's current `main.py` and save a backup outside
   this repository. That backup may contain Wi-Fi credentials, so do not place
   it under the `robot-1` directory.
2. Copy the repository's `pico_relay.py` to a temporary file outside the
   repository.
3. Set `SSID` and `PASSWORD` only in that temporary copy.
4. Save the temporary copy to the Pico W as `main.py`; do not overwrite the
   repository's existing UART-probe `pico/main.py`.
5. Restart the Pico W and confirm `Wi-Fi OK`, `listening on ...:9000`, and—only
   after an approved STOP-only host connection—`relay ready`.
6. Keep the backup until the STOP-only check has passed. Restore it from Thonny
   if the revised relay does not start.

Never use Thonny's “save to computer” operation with the repository
`pico/main.py` as the destination: doing so could overwrite the retained UART
probe and accidentally store credentials in Git.

For the STOP-only diagnostic deployment, verify before upload that the local
copy retains both settings below:

```python
COMMAND_TIMEOUT_MS = 500
ALLOW_NONZERO_DRIVE = False

# 非ゼロ走行は、このロックを明示的に変更したうえで、ROS 2側から
# 世代番号付きARMを成功させたセッションでのみ許可されます。
```

## ラッチ式watchdogとARM

リレーの走行状態は `STOPPED`、`ARMED`、`LATCHED_STOP` の3状態です。
watchdog、TCP切断、例外ではSTOPを書き込んで `LATCHED_STOP` になります。
通常STOP、再接続、診断要求だけではラッチは解除されません。

非ゼロDrive DirectはARM世代番号付きの固定フレームだけを受理します。
旧5バイト形式はSTOPとの互換性だけを維持し、非ゼロ値は拒否します。
ARMも `ALLOW_NONZERO_DRIVE=False` のままでは拒否されます。

ホスト側も `drive_enabled=true`、STOP ACK、OI Safe確認を通過した後に、
独立したARMトピックへ単調増加する世代番号を1回送る必要があります。

```bash
GENERATION=$(date +%s)
ros2 topic pub --once /roomba/arm_generation std_msgs/msg/UInt32 \
  "{data: ${GENERATION}}"
```

再接続後やwatchdog発火後は、以前より大きい新しい世代番号でARMし直します。
ARMは走行指令ではありませんが、両輪浮上、二重ロック、Safe診断が揃うまで
実行しないでください。

The TCP protocol keeps the legacy five-byte Drive Direct frame for STOP
compatibility. Diagnostic frames use the fixed `D7 01` (STOP verification) and
`D7 02` (OI Mode packet 35) requests only. No frame permits an arbitrary OI
opcode or arbitrary UART payload. Diagnostics always issue STOP first and do
not refresh the motion watchdog.

`roomba_uart_probe.py` sends only the Roomba Open Interface `Start` command and
three fixed Query List requests for charging state, voltage, charge/capacity,
charging source, OI mode, and wheel-drop bits. It does not enable Safe/Full mode
or send Drive, cleaning, reset, Wi-Fi, or automatic-start commands. Voltage is
reported without a pass/fail threshold until the installed battery type and
rating have been checked.

## Wiring

Turn off and disconnect power before changing any wiring.

| Pico W / level converter | Roomba Mini-DIN |
|---|---|
| GP4 (UART1 TX) -> LV1 -> HV1 | Actual pin 3 (RXD) |
| GP5 (UART1 RX) <- LV2 <- HV2 | Actual pin 4 (TXD) |
| GND | GND (common with Pico and converter) |
| 3V3 OUT -> LV supply | - |
| Mobile-battery USB 5 V -> Pico W power | - |

- Do not use the battery power on Roomba pins 1 or 2.
- Never connect a 5 V signal directly to a Pico GPIO.
- Identify actual Mini-DIN pin numbers from the connector orientation; do not
  infer pin numbers from cable colors or a mirrored view.
- CP2102 and FLINT are not used for this test.

## Run with MicroPico

1. With power off, complete and recheck the wiring above.
2. For bench-only probe work, power the Pico W from the Mac USB, wake the Roomba,
   and leave the Roomba stopped. The final demo instead uses the mobile battery
   and does not connect Pico USB to the Mac.
3. Open `roomba_uart_probe.py` in VS Code.
4. Use **Run Current File on Board** and watch the USB REPL output.

## Final demo power and communication topology

The submitted hardware configuration is fixed as follows:

```text
MacBook -- Wi-Fi / TCP 9000 --> Pico W -- UART1 GP4/GP5 --> Roomba 680
                                  ^
                                  +-- mobile-battery USB 5 V
```

Do not connect the Pico W to the Mac by USB during the final demo. Deploy a
credential-filled local copy of the current `pico_relay.py` as `main.py`; the
legacy repository `pico/main.py` is not the canonical final-demo relay. Pico
evidence comes from fixed TCP ACKs, ROS 2 bridge logs, the post-run STOP/OI
diagnostic, and the physical-stop video. UDP diagnostics are deferred until
after submission.

5. The program makes three checks and exits by itself. Use `Ctrl+C` if you need
   to stop it early; the UART is deinitialized in either case.

The default is 115200 bps. To test a Roomba already configured for 19200 bps,
manually change `BAUDRATE = 115200` to `BAUDRATE = 19200` before running it.
The program does not switch rates automatically.

This code has not been verified with the physical Pico W and Roomba in this
workspace.

## Wi-Fi起動・安定性の計測（Macのみ）

TCP 9000、UART、走行ロックには触れず、ICMP pingだけを1秒間隔で記録する。計測コマンドを
電源投入直前に開始し、表示された`start ... power on`の直後にPicoへ給電する。

```bash
cd /Users/imaichika/robot
.venv/bin/python pico_ping_monitor.py --host 192.168.0.29 \
  --label mac-usb --duration 300

.venv/bin/python pico_ping_monitor.py --host 192.168.0.29 \
  --label mobile-battery --duration 300
```

各試行は`artifacts/pico_ping/`へ時刻付きCSVと集計JSONを保存する。比較する値は
`first_response_seconds`、`longest_consecutive_failures`、`response_rate_percent`、
`average_latency_ms`である。公平な比較のため、Picoの位置、Wi-Fi AP、測定時間、Macの位置を
揃え、各給電方式を最低3回ずつ測る。これは疎通計測だけで、TCP診断や走行許可を行わない。
