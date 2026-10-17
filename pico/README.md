# Pico W / Roomba UART probe

## File roles

- `main.py`: the existing manual UART probe; retained without replacing it.
- `roomba_uart_probe.py`: read-only battery-voltage probe.
- `pico_relay.py`: canonical Wi-Fi TCP-to-UART relay source. For deployment,
  save a local copy of this file on the Pico W as `main.py`.

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
three battery-voltage sensor requests. It does not enable Safe/Full mode or send
any drive, cleaning, reset, BRC, Wi-Fi, or automatic-start command.

## Wiring

Turn off and disconnect power before changing any wiring.

| Pico W / level converter | Roomba Mini-DIN |
|---|---|
| GP0 (UART0 TX) -> LV1 -> HV1 | Actual pin 3 (RXD) |
| GP1 (UART0 RX) <- LV2 <- HV2 | Actual pin 4 (TXD) |
| GND | GND (common with Pico and converter) |
| 3V3 OUT -> LV supply | - |
| USB-derived VBUS 5 V -> HV supply | - |

- Do not use the battery power on Roomba pins 1 or 2.
- Never connect a 5 V signal directly to a Pico GPIO.
- Identify actual Mini-DIN pin numbers from the connector orientation; do not
  infer pin numbers from cable colors or a mirrored view.
- CP2102 and FLINT are not used for this test.

## Run with MicroPico

1. With power off, complete and recheck the wiring above.
2. Power the Pico W by USB, wake the Roomba, and leave the Roomba stopped.
3. Open `roomba_uart_probe.py` in VS Code.
4. Use **Run Current File on Board** and watch the USB REPL output.
5. The program makes three checks and exits by itself. Use `Ctrl+C` if you need
   to stop it early; the UART is deinitialized in either case.

The default is 115200 bps. To test a Roomba already configured for 19200 bps,
manually change `BAUDRATE = 115200` to `BAUDRATE = 19200` before running it.
The program does not switch rates automatically.

This code has not been verified with the physical Pico W and Roomba in this
workspace.
