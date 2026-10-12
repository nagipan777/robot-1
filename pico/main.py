"""Roomba 680 通信テスト (Pico / MicroPython, 19200bps)

配線:
    Pico GP0 (TX) -> Roomba Mini-DIN ピン3 (RXD)
    Pico GP1 (RX) <- Roomba Mini-DIN ピン4 (TXD)
    Pico GND      -- Roomba Mini-DIN ピン6か7 (GND)
    ※ ピン1・2 (約14V) には絶対に繋がない

使い方:
    CLEANボタンでルンバを起こしてから Thonny で実行
"""
import machine
import time

BAUD = 19200  # あなたの680で動いた値

START = bytes([128])
SAFE = bytes([131])
SENSORS_BATTERY_VOLTAGE = bytes([142, 22])

uart = machine.UART(
    0,
    baudrate=BAUD,
    bits=8,
    parity=None,
    stop=1,
    tx=machine.Pin(0),
    rx=machine.Pin(1),
)


def drain():
    time.sleep_ms(100)
    while uart.any():
        uart.read(uart.any())
        time.sleep_ms(10)


def read_exact(size, timeout_ms=2000):
    buf = bytearray()
    t0 = time.ticks_ms()
    while len(buf) < size and time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        n = uart.any()
        if n:
            buf.extend(uart.read(min(n, size - len(buf))))
        else:
            time.sleep_ms(10)
    return bytes(buf)


try:
    print("Roomba test @ {} bps".format(BAUD))
    drain()

    uart.write(START)
    time.sleep_ms(300)
    drain()

    for i in range(3):
        uart.write(SENSORS_BATTERY_VOLTAGE)
        data = read_exact(2)
        print("[{}] raw: {}".format(i + 1, " ".join("{:02X}".format(b) for b in data) or "<empty>"))
        if len(data) == 2:
            mv = (data[0] << 8) | data[1]
            print("    battery: {} mV".format(mv))
        time.sleep_ms(500)
finally:
    uart.deinit()
    print("done")
    