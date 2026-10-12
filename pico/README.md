# Pico W / Roomba UART probe

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
