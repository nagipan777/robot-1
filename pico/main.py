"""Pico W: Wi-Fi (TCP) to Roomba 680 UART relay with fail-safe reconnects."""

import errno
import machine
import network
import socket
import time


# Set these only on the deployed Pico W. Never commit Wi-Fi credentials.
SSID = ""
PASSWORD = ""
PORT = 9000

BAUD = 115200
COMMAND_TIMEOUT_MS = 500
WIFI_CONNECT_TIMEOUT_MS = 30000
WIFI_RETRY_MS = 3000
ACCEPT_TIMEOUT_S = 1
UART_DIAGNOSTIC_TIMEOUT_MS = 1000
UART_DRAIN_QUIET_MS = 10
UART_DRAIN_TIMEOUT_MS = 100
TCP_SEND_TIMEOUT_MS = 100
ALLOW_NONZERO_DRIVE = False

STOP = bytes([145, 0, 0, 0, 0])
READY = b"READY\n"
DRIVE_DIRECT = 145
DRIVE_DIRECT_SIZE = 5
OI_MAX_WHEEL_SPEED_MM_S = 500
DIAGNOSTIC_MAGIC = 0xD7
DIAGNOSTIC_STOP = 0x01
DIAGNOSTIC_OI_MODE = 0x02
DIAGNOSTIC_OI_MODE_LIVE = 0x03
CONTROL_ARM = 0x04
GENERATION_DRIVE = 0x05
SEQUENCED_DRIVE = 0x06
DIAGNOSTIC_STOP_RESPONSE = 0x81
DIAGNOSTIC_OI_MODE_RESPONSE = 0x82
CONTROL_ARM_RESPONSE = 0x84
SEQUENCED_DRIVE_RESPONSE = 0x86
DIAGNOSTIC_ERROR = 0xE0
ARM_ACCEPTED = 0x01
ARM_REJECTED_LOCK = 0x02
ARM_REJECTED_GENERATION = 0x03
DRIVE_ACK_VALIDATED = 0x01
DRIVE_ACK_UART_WRITTEN = 0x02
STOP_ACK_VALIDATED = 0x01
STOP_ACK_UART_WRITTEN = 0x02
STOP_ACK_ROBOT_ZERO = 0x04
SENSORS = 142
QUERY_LIST = 149
PAUSE_RESUME_STREAM = 150
PAUSE_STREAM = bytes([PAUSE_RESUME_STREAM, 0])
SENSOR_OI_MODE = 35
SENSOR_REQUESTED_RIGHT_VELOCITY = 41
SENSOR_REQUESTED_LEFT_VELOCITY = 42
DIAGNOSTIC_SENTINEL = 32767
CONTROL_ARM_SIZE = 6
GENERATION_DRIVE_SIZE = 11
SEQUENCED_DRIVE_SIZE = 17
WOULD_BLOCK = tuple(
    value
    for value in (
        11,
        -11,
        35,
        -35,
        getattr(errno, "EAGAIN", None),
        getattr(errno, "EWOULDBLOCK", None),
    )
    if value is not None
)

uart = machine.UART(
    1,
    baudrate=BAUD,
    bits=8,
    parity=None,
    stop=1,
    tx=machine.Pin(4),
    rx=machine.Pin(5),
)
brc = machine.Pin(2, machine.Pin.OUT, value=1)
led = machine.Pin("LED", machine.Pin.OUT)
led.off()
wlan = network.WLAN(network.STA_IF)


class MotionLatch:
    STOPPED = "STOPPED"
    ARMED = "ARMED"
    LATCHED_STOP = "LATCHED_STOP"

    def __init__(self):
        self.state = self.STOPPED
        self.generation = None
        self.highest_generation = 0
        self.last_sequence = 0

    def arm(self, generation):
        if generation <= self.highest_generation:
            return False
        self.highest_generation = generation
        self.generation = generation
        self.last_sequence = 0
        self.state = self.ARMED
        return True

    def accepts(self, generation, sequence):
        return (
            self.state == self.ARMED
            and generation == self.generation
            and sequence > self.last_sequence
        )

    def record(self, sequence):
        self.last_sequence = sequence

    def normal_stop(self):
        if self.state == self.ARMED:
            self.state = self.STOPPED
            self.generation = None

    def latch_stop(self):
        self.state = self.LATCHED_STOP
        self.generation = None


motion_latch = MotionLatch()


def stop_roomba():
    uart.write(STOP)


def uart_write_exact(data):
    written = uart.write(data)
    return written == len(data)


def wifi_connect():
    """Connect within a finite time and return True; leave retry policy to caller."""
    wlan.active(True)
    try:
        wlan.config(pm=0xA11140)
    except Exception:
        pass

    if wlan.isconnected():
        print("Wi-Fi OK; IP:", wlan.ifconfig()[0])
        led.on()
        return True

    print("connecting Wi-Fi...")
    wlan.connect(SSID, PASSWORD)
    started = time.ticks_ms()
    while not wlan.isconnected():
        if time.ticks_diff(time.ticks_ms(), started) >= WIFI_CONNECT_TIMEOUT_MS:
            print("Wi-Fi timeout; retrying")
            wlan.disconnect()
            led.off()
            return False
        time.sleep_ms(250)

    print("Wi-Fi OK; IP:", wlan.ifconfig()[0])
    led.on()
    return True


def wait_for_wifi():
    stop_roomba()
    while not wifi_connect():
        time.sleep_ms(WIFI_RETRY_MS)


def roomba_init():
    brc.off()
    time.sleep_ms(200)
    brc.on()
    time.sleep_ms(500)
    uart.write(bytes([128]))
    time.sleep_ms(200)
    uart.write(bytes([131]))
    time.sleep_ms(200)
    uart_write_exact(PAUSE_STREAM)
    time.sleep_ms(20)
    if not drain_uart_until_quiet():
        # Keep TCP diagnostics available, but do not claim that UART sensor
        # responses are synchronized.  STOP/OI diagnostics will retry the
        # bounded drain and the host Safe gate remains closed on failure.
        print("UART not quiet after OI init; diagnostics remain locked")


def signed_16(msb, lsb):
    value = (msb << 8) | lsb
    return value - 65536 if value >= 32768 else value


def encoded_16(value):
    value &= 0xFFFF
    return bytes([(value >> 8) & 0xFF, value & 0xFF])


def encoded_32(value):
    return bytes(
        [
            (value >> 24) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ]
    )


def unsigned_32(data):
    return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]


def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (
                ((crc << 1) ^ 0x1021) & 0xFFFF
                if crc & 0x8000
                else (crc << 1) & 0xFFFF
            )
    return crc


def send_drive_ack(connection, status, generation, sequence):
    payload = (
        bytes([DIAGNOSTIC_MAGIC, SEQUENCED_DRIVE_RESPONSE, status])
        + encoded_32(generation)
        + encoded_32(sequence)
    )
    send_all(connection, payload + encoded_16(crc16_ccitt(payload)))


def hex_bytes(data):
    """Format fixed OI diagnostic bytes without exposing Wi-Fi credentials."""
    return " ".join("{:02x}".format(value) for value in data)


def send_all(connection, data):
    offset = 0
    started = time.ticks_ms()
    while offset < len(data):
        try:
            sent = connection.send(data[offset:])
        except OSError as error:
            code = error.args[0] if error.args else None
            if code not in WOULD_BLOCK:
                raise
            if time.ticks_diff(time.ticks_ms(), started) >= TCP_SEND_TIMEOUT_MS:
                raise OSError("TCP send timed out")
            time.sleep_ms(1)
            continue
        if sent is None:
            sent = len(data) - offset
        if sent <= 0:
            raise OSError("TCP send returned {}".format(sent))
        offset += sent


def drain_uart_until_quiet(
    quiet_ms=UART_DRAIN_QUIET_MS, timeout_ms=UART_DRAIN_TIMEOUT_MS
):
    """Discard stale UART bytes and require a bounded quiet interval."""
    started = time.ticks_ms()
    quiet_started = started
    while time.ticks_diff(time.ticks_ms(), started) < timeout_ms:
        available = uart.any()
        if available:
            uart.read(available)
            quiet_started = time.ticks_ms()
        elif time.ticks_diff(time.ticks_ms(), quiet_started) >= quiet_ms:
            return True
        time.sleep_ms(2)
    return False


def read_uart_exact(size, timeout_ms=UART_DIAGNOSTIC_TIMEOUT_MS):
    received = bytearray()
    started = time.ticks_ms()
    while len(received) < size:
        available = uart.any()
        if available:
            received.extend(uart.read(min(available, size - len(received))))
            continue
        if time.ticks_diff(time.ticks_ms(), started) >= timeout_ms:
            break
        time.sleep_ms(2)
    return bytes(received)


def query_uart(command, response_size):
    # A stream started by an earlier client can continuously inject framed
    # bytes into these headerless query responses.  OI opcode 150, state 0,
    # pauses only the stream and does not change Passive/Safe/Full mode.
    if not uart_write_exact(PAUSE_STREAM):
        print("UART diagnostic pause write failed")
        return b""
    time.sleep_ms(20)
    if not drain_uart_until_quiet():
        print("UART diagnostic sync failed; command={}".format(hex_bytes(command)))
        return b""
    if not uart_write_exact(command):
        print("UART diagnostic query write failed; command={}".format(hex_bytes(command)))
        return b""
    response = read_uart_exact(response_size)
    if len(response) != response_size:
        print(
            "UART diagnostic response incomplete: received {}/{}; raw={}".format(
                len(response), response_size, hex_bytes(response)
            )
        )
    return response


def handle_stop_diagnostic(connection):
    status = STOP_ACK_VALIDATED
    requested_right = DIAGNOSTIC_SENTINEL
    requested_left = DIAGNOSTIC_SENTINEL

    if uart_write_exact(STOP):
        status |= STOP_ACK_UART_WRITTEN
        response = query_uart(
            bytes(
                [
                    QUERY_LIST,
                    2,
                    SENSOR_REQUESTED_RIGHT_VELOCITY,
                    SENSOR_REQUESTED_LEFT_VELOCITY,
                ]
            ),
            4,
        )
        if len(response) == 4:
            requested_right = signed_16(response[0], response[1])
            requested_left = signed_16(response[2], response[3])
            if requested_right == 0 and requested_left == 0:
                status |= STOP_ACK_ROBOT_ZERO

    payload = bytes([DIAGNOSTIC_MAGIC, DIAGNOSTIC_STOP_RESPONSE, status])
    payload += encoded_16(requested_right) + encoded_16(requested_left)
    send_all(connection, payload)


def handle_oi_mode_diagnostic(connection, stop_first=True):
    if stop_first:
        # Initial diagnostics remain STOP-first.  The live variant is a fixed
        # sensor-35 query used only while the host is actively supervising
        # motion; it cannot carry an arbitrary OI command or refresh watchdog.
        uart_write_exact(STOP)
    response = query_uart(bytes([SENSORS, SENSOR_OI_MODE]), 1)
    if len(response) == 1 and response[0] in (0, 1, 2, 3):
        send_all(
            connection,
            bytes([DIAGNOSTIC_MAGIC, DIAGNOSTIC_OI_MODE_RESPONSE, response[0]]),
        )
        return
    send_all(connection, bytes([DIAGNOSTIC_MAGIC, DIAGNOSTIC_ERROR, 1]))


def handle_arm(connection, generation):
    if not ALLOW_NONZERO_DRIVE:
        status = ARM_REJECTED_LOCK
        print("ARM rejected: nonzero drive lock; generation={}".format(generation))
    elif motion_latch.arm(generation):
        status = ARM_ACCEPTED
        print("ARM accepted: generation={}".format(generation))
    else:
        status = ARM_REJECTED_GENERATION
        print(
            "ARM rejected: stale generation={} highest={}".format(
                generation, motion_latch.highest_generation
            )
        )
    send_all(
        connection,
        bytes([DIAGNOSTIC_MAGIC, CONTROL_ARM_RESPONSE, status])
        + encoded_32(motion_latch.highest_generation),
    )
    return status == ARM_ACCEPTED


def forward_valid_frames(received, connection):
    """Process allow-listed frames; return unconsumed bytes and drive count."""
    forwarded = 0
    offset = 0
    received_size = len(received)
    while offset < received_size:
        opcode = received[offset]
        if opcode == DIAGNOSTIC_MAGIC:
            if received_size - offset < 2:
                break
            diagnostic = received[offset + 1]
            if diagnostic == CONTROL_ARM:
                if received_size - offset < CONTROL_ARM_SIZE:
                    break
                generation = unsigned_32(received[offset + 2 : offset + 6])
                offset += CONTROL_ARM_SIZE
                handle_arm(connection, generation)
                continue
            if diagnostic == GENERATION_DRIVE:
                if received_size - offset < GENERATION_DRIVE_SIZE:
                    break
                generation = unsigned_32(received[offset + 2 : offset + 6])
                frame = bytes(received[offset + 6 : offset + 11])
                offset += GENERATION_DRIVE_SIZE
                right = signed_16(frame[1], frame[2])
                left = signed_16(frame[3], frame[4])
                if frame[0] != DRIVE_DIRECT or right == 0 and left == 0:
                    print("drive rejected: invalid generation frame")
                    continue
                print("drive rejected: sequenced CRC frame required")
                continue
            if diagnostic == SEQUENCED_DRIVE:
                if received_size - offset < SEQUENCED_DRIVE_SIZE:
                    break
                raw = bytes(received[offset : offset + SEQUENCED_DRIVE_SIZE])
                offset += SEQUENCED_DRIVE_SIZE
                payload = raw[:-2]
                supplied_crc = (raw[-2] << 8) | raw[-1]
                generation = unsigned_32(raw[2:6])
                sequence = unsigned_32(raw[6:10])
                frame = raw[10:15]
                status = 0
                if crc16_ccitt(payload) != supplied_crc or frame[0] != DRIVE_DIRECT:
                    print("drive rejected: invalid CRC or opcode")
                    send_drive_ack(connection, status, generation, sequence)
                    continue
                status |= DRIVE_ACK_VALIDATED
                right = signed_16(frame[1], frame[2])
                left = signed_16(frame[3], frame[4])
                if (
                    right == 0 and left == 0
                    or abs(right) > OI_MAX_WHEEL_SPEED_MM_S
                    or abs(left) > OI_MAX_WHEEL_SPEED_MM_S
                    or not ALLOW_NONZERO_DRIVE
                    or not motion_latch.accepts(generation, sequence)
                ):
                    print(
                        "drive rejected: lock/generation/sequence generation={} sequence={}".format(
                            generation, sequence
                        )
                    )
                    send_drive_ack(connection, status, generation, sequence)
                    continue
                if uart_write_exact(frame):
                    status |= DRIVE_ACK_UART_WRITTEN
                    motion_latch.record(sequence)
                    forwarded += 1
                    print(
                        "drive forwarded: at_ms={} generation={} sequence={} right={} left={}".format(
                            time.ticks_ms(), generation, sequence, right, left
                        )
                    )
                else:
                    stop_roomba()
                    motion_latch.latch_stop()
                    print("incomplete UART Drive Direct write; STOP latched")
                send_drive_ack(connection, status, generation, sequence)
                continue
            offset += 2
            if diagnostic == DIAGNOSTIC_STOP:
                handle_stop_diagnostic(connection)
            elif diagnostic == DIAGNOSTIC_OI_MODE:
                handle_oi_mode_diagnostic(connection)
            elif diagnostic == DIAGNOSTIC_OI_MODE_LIVE:
                handle_oi_mode_diagnostic(connection, stop_first=False)
            else:
                send_all(
                    connection,
                    bytes([DIAGNOSTIC_MAGIC, DIAGNOSTIC_ERROR, diagnostic]),
                )
            continue
        if opcode != DRIVE_DIRECT:
            offset += 1
            continue
        if received_size - offset < DRIVE_DIRECT_SIZE:
            break

        frame = bytes(received[offset : offset + DRIVE_DIRECT_SIZE])
        offset += DRIVE_DIRECT_SIZE
        right = signed_16(frame[1], frame[2])
        left = signed_16(frame[3], frame[4])
        if (
            abs(right) > OI_MAX_WHEEL_SPEED_MM_S
            or abs(left) > OI_MAX_WHEEL_SPEED_MM_S
        ):
            print("rejected out-of-range Drive Direct frame")
            continue
        if not ALLOW_NONZERO_DRIVE and (right != 0 or left != 0):
            print("rejected non-zero Drive Direct frame")
            continue
        if right != 0 or left != 0:
            print("drive rejected: generation required")
            continue
        if not uart_write_exact(frame):
            print("incomplete UART Drive Direct write")
            continue
        forwarded_at = time.ticks_ms()
        if right == 0 and left == 0:
            motion_latch.normal_stop()
            print("normal STOP forwarded: at_ms={}".format(forwarded_at))
    return bytearray(received[offset:]), forwarded


def enforce_command_watchdog(last_command, stopped):
    """Write an OI STOP once when valid drive traffic has expired."""
    fired_at = time.ticks_ms()
    elapsed = time.ticks_diff(fired_at, last_command)
    if stopped or elapsed < COMMAND_TIMEOUT_MS:
        return stopped
    uart_written = uart_write_exact(STOP)
    motion_latch.latch_stop()
    print(
        "watchdog STOP: fired_at_ms={} last_drive_at_ms={} "
        "elapsed_ms={} uart_written={}".format(
            fired_at, last_command, elapsed, uart_written
        )
    )
    return True


def make_server():
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", PORT))
    server.listen(1)
    server.settimeout(ACCEPT_TIMEOUT_S)
    print("listening on {}:{}".format(wlan.ifconfig()[0], PORT))
    return server


def receive_client(connection, address):
    connected_at = time.ticks_ms()
    print("connected: {} at_ms={}".format(address, connected_at))
    disconnect_reason = "PC closed the connection"

    try:
        # Keep the socket blocking until OI initialization is complete.  The host
        # must not send wheel commands before receiving this readiness marker.
        roomba_init()
        send_all(connection, READY)
        print("relay ready: at_ms={}".format(time.ticks_ms()))
        connection.setblocking(False)

        last_command = time.ticks_ms()
        stopped = True
        received = bytearray()
        while wlan.isconnected():
            try:
                data = connection.recv(256)
                if data == b"":
                    break
                if data:
                    received.extend(data)
                    received, forwarded = forward_valid_frames(received, connection)
                    if forwarded:
                        # Invalid, incomplete, diagnostic, ARM, and legacy STOP
                        # data cannot extend motion. Only a CRC-checked,
                        # sequence-checked Drive written fully to UART can do so.
                        last_command = time.ticks_ms()
                        stopped = False
            except OSError as error:
                code = error.args[0] if error.args else None
                if code not in WOULD_BLOCK:
                    disconnect_reason = "TCP receive error: {}".format(error)
                    break

            stopped = enforce_command_watchdog(last_command, stopped)

            time.sleep_ms(5)
        if not wlan.isconnected():
            disconnect_reason = "Wi-Fi disconnected"
    except OSError as error:
        disconnect_reason = "client setup error: {}".format(error)
    finally:
        stop_roomba()
        motion_latch.latch_stop()
        connection.close()
        print(
            "disconnect STOP: at_ms={} reason={}".format(
                time.ticks_ms(), disconnect_reason
            )
        )


def serve():
    while True:
        wait_for_wifi()
        server = None
        try:
            server = make_server()
            while wlan.isconnected():
                try:
                    connection, address = server.accept()
                except OSError:
                    # The finite accept timeout lets us notice a Wi-Fi / DHCP loss.
                    continue
                receive_client(connection, address)
        except OSError as error:
            print("server error:", error)
        finally:
            stop_roomba()
            if server is not None:
                server.close()
            led.off()

        print("Wi-Fi lost; rebuilding listener")
        time.sleep_ms(WIFI_RETRY_MS)


def main():
    try:
        serve()
    except KeyboardInterrupt:
        print("stopped by user")
    finally:
        stop_roomba()
        uart.deinit()
        led.off()
        print("UART closed; Roomba stopped")


if __name__ == "__main__":
    main()
