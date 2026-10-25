"""Pico W TCP velocity logger. Deliberately contains no UART/Roomba access."""

import network
import socket
import time

from dry_run_protocol import DryRunReceiverState, ProtocolError, encode_ack, encode_ready


# Set these only on the deployed Pico W. Never commit Wi-Fi credentials.
SSID = ""
PASSWORD = ""
PORT = 9001
RECEIVE_TIMEOUT_MS = 500
STOPPED_IDLE_TIMEOUT_MS = 2000


def now_ms():
    return time.ticks_ms()


def elapsed_ms(now, then):
    return time.ticks_diff(now, then)


def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID, PASSWORD)
    while not wlan.isconnected():
        time.sleep_ms(250)
    print("dry-run Wi-Fi OK; IP:", wlan.ifconfig()[0])


def serve_connection(connection, session_id):
    state = DryRunReceiverState(session_id, RECEIVE_TIMEOUT_MS)
    connected_at = now_ms()
    connection.settimeout(0.1)
    connection.sendall(encode_ready(session_id))
    buffer = bytearray()
    print("dry-run client connected; session={}; state=STOPPED; UART=DISABLED".format(session_id))
    try:
        while True:
            try:
                chunk = connection.recv(64)
                if not chunk:
                    break
                buffer.extend(chunk)
                if len(buffer) > 160:
                    state.stop("invalid_frame")
                    print("dry-run STOPPED reason=invalid_frame")
                    buffer = bytearray()
                while b"\n" in buffer:
                    line, remainder = bytes(buffer).split(b"\n", 1)
                    buffer = bytearray(remainder)
                    try:
                        sequence, status = state.accept(line + b"\n", now_ms())
                        print(
                            "dry-run seq={} state={} linear_mm_s={} angular_mrad_s={}".format(
                                sequence, status, state.linear_mm_s, state.angular_mrad_s
                            )
                        )
                        connection.sendall(encode_ack(session_id, sequence, status))
                    except (ProtocolError, ValueError) as exc:
                        print("dry-run STOPPED reason=invalid_frame error={}".format(exc))
            except OSError:
                pass
            reference = state.last_received_ms
            age = elapsed_ms(now_ms(), reference if reference is not None else connected_at)
            timeout_ms = (
                state.timeout_ms
                if reference is None or state.state == "ACTIVE"
                else STOPPED_IDLE_TIMEOUT_MS
            )
            if age > timeout_ms:
                if reference is None:
                    reason = "handshake_timeout"
                elif state.state == "ACTIVE":
                    reason = "receive_timeout"
                else:
                    reason = "stopped_idle_timeout"
                state.stop(reason)
                print("dry-run STOPPED reason={} age_ms={} session={}".format(reason, age, session_id))
                break
    finally:
        state.disconnect()
        print("dry-run STOPPED reason=communication_lost; session={}; UART=DISABLED".format(session_id))
        connection.close()


def main():
    connect_wifi()
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", PORT))
    server.listen(1)
    print("dry-run log receiver listening on port {}; UART=DISABLED".format(PORT))
    session_id = 0
    while True:
        connection, _address = server.accept()
        session_id += 1
        serve_connection(connection, session_id)


main()
