import machine
import time


# Change this manually to 19200 if the Roomba is configured for that rate.
# This program never changes the baud rate automatically.
BAUDRATE = 19200

RESPONSE_TIMEOUT_MS = 2000
DRAIN_TIMEOUT_MS = 200
EXPECTED_MIN_MV = 10000
EXPECTED_MAX_MV = 18000

START = bytes([128])
SENSORS_BATTERY_VOLTAGE = bytes([142, 22])


def format_hex(data):
    if not data:
        return "<empty>"
    return " ".join("{:02X}".format(value) for value in data)


def drain_input(uart):
    """Discard pending bytes for a bounded amount of time."""
    drained = 0
    started = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), started) < DRAIN_TIMEOUT_MS:
        available = uart.any()
        if available:
            data = uart.read(available)
            if data:
                drained += len(data)
        else:
            time.sleep_ms(10)

    return drained


def read_exact(uart, size, timeout_ms):
    """Read up to size bytes, allowing the response to arrive in pieces."""
    received = bytearray()
    started = time.ticks_ms()

    while len(received) < size:
        if time.ticks_diff(time.ticks_ms(), started) >= timeout_ms:
            break

        available = uart.any()
        if available:
            data = uart.read(min(available, size - len(received)))
            if data:
                received.extend(data)
        else:
            time.sleep_ms(10)

    return bytes(received)


def report_voltage_response(response):
    print("  Raw response:", format_hex(response))

    if len(response) == 0:
        print("  Result: no response (0 of 2 bytes received)")
        return

    if len(response) == 1:
        print("  Result: partial response (1 of 2 bytes received)")
        return

    millivolts = (response[0] << 8) | response[1]
    volts = millivolts / 1000

    if EXPECTED_MIN_MV <= millivolts <= EXPECTED_MAX_MV:
        print("  Result: valid 2-byte response")
        print("  Battery voltage: {} mV ({:.3f} V)".format(millivolts, volts))
    else:
        print("  Result: 2-byte response, but voltage is outside the expected range")
        print("  Battery voltage: {} mV ({:.3f} V)".format(millivolts, volts))


uart = machine.UART(
    0,
    baudrate=BAUDRATE,
    bits=8,
    parity=None,
    stop=1,
    tx=machine.Pin(0),
    rx=machine.Pin(1),
)

brc = machine.Pin(2, machine.Pin.OUT, value=1)  # GP2 → Mini-DIN ピン5(BRC)
brc.value(0); time.sleep_ms(200); brc.value(1)
time.sleep_ms(500)

try:
    print("Roomba UART probe: {} bps, 8N1".format(BAUDRATE))
    drain_input(uart)
    uart.write(START)
    print("Start command sent; waiting 500 ms before sensor requests")
    time.sleep_ms(500)

    for attempt in range(1, 4):
        print("Request {}/3".format(attempt))

        drained = drain_input(uart)
        if drained:
            print("  Discarded {} stale byte(s) before request".format(drained))

        uart.write(SENSORS_BATTERY_VOLTAGE)
        response = read_exact(uart, 2, RESPONSE_TIMEOUT_MS)
        report_voltage_response(response)

        if attempt < 3:
            time.sleep_ms(500)
finally:
    uart.deinit()
    print("UART deinitialized")
