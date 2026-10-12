from machine import UART, Pin
import time

uart = UART(
    0,
    baudrate=19200,
    bits=8,
    parity=None,
    stop=1,
    rx=Pin(1)
)

print("UART RX monitor 19200 ready")

while True:
    if uart.any():
        data = uart.read()
        if data:
            print("RX:", data.hex())
    time.sleep_ms(10)