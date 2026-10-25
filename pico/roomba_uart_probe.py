import machine
import time

from roomba_diagnostic_codec import decode_diagnostic_response


# Change this manually to 19200 if the Roomba is configured for that rate.
# This program never changes the baud rate automatically.
BAUDRATE = 115200

RESPONSE_TIMEOUT_MS = 2000
DRAIN_TIMEOUT_MS = 200

START = bytes([128])
QUERY_LIST = 149
SENSOR_CHARGING_STATE = 21
SENSOR_VOLTAGE = 22
SENSOR_BATTERY_CHARGE = 25
SENSOR_BATTERY_CAPACITY = 26
SENSOR_CHARGING_SOURCES = 34
SENSOR_OI_MODE = 35
SENSOR_BUMPS_AND_WHEEL_DROPS = 7
DIAGNOSTIC_QUERY = bytes(
    [
        QUERY_LIST,
        7,
        SENSOR_CHARGING_STATE,
        SENSOR_VOLTAGE,
        SENSOR_BATTERY_CHARGE,
        SENSOR_BATTERY_CAPACITY,
        SENSOR_CHARGING_SOURCES,
        SENSOR_OI_MODE,
        SENSOR_BUMPS_AND_WHEEL_DROPS,
    ]
)


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


def report_diagnostic_response(response):
    print("  Raw response:", format_hex(response))
    try:
        status = decode_diagnostic_response(response)
    except (TypeError, ValueError) as exc:
        print("  Result: invalid response ({})".format(exc))
        return False
    print(
        "  Charging: {} ({})".format(
            status["charging_state"], status["charging_state_name"]
        )
    )
    print(
        "  Battery voltage: {} mV ({:.3f} V); informational only".format(
            status["voltage_mv"], status["voltage_mv"] / 1000
        )
    )
    print(
        "  Battery charge/capacity: {} / {} mAh ({}%)".format(
            status["charge_mah"],
            status["capacity_mah"],
            "unknown"
            if status["charge_percent"] is None
            else "{:.1f}".format(status["charge_percent"]),
        )
    )
    print(
        "  Charging source: internal={} home_base={}".format(
            status["charging_source_internal"],
            status["charging_source_home_base"],
        )
    )
    print("  OI mode: {} ({})".format(status["oi_mode"], status["oi_mode_name"]))
    print(
        "  Wheel drop: left={} right={}".format(
            status["wheel_drop_left"],
            status["wheel_drop_right"],
        )
    )
    return True


uart = machine.UART(
    1,
    baudrate=BAUDRATE,
    bits=8,
    parity=None,
    stop=1,
    tx=machine.Pin(4),
    rx=machine.Pin(5),
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

        uart.write(DIAGNOSTIC_QUERY)
        response = read_exact(uart, 10, RESPONSE_TIMEOUT_MS)
        report_diagnostic_response(response)

        if attempt < 3:
            time.sleep_ms(500)
finally:
    uart.deinit()
    print("UART deinitialized")
