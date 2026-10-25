"""ROS-independent TCP client for the Pico W log-only receiver."""

import socket

from dry_run_protocol import decode_ack, decode_ready, encode_command


class ReconnectBackoff:
    def __init__(self, initial_delay=1.0, max_delay=8.0, multiplier=2.0):
        self.initial_delay = initial_delay
        self.max_delay = max_delay
        self.multiplier = multiplier
        self.current_delay = initial_delay

    def failed(self):
        delay = self.current_delay
        self.current_delay = min(self.max_delay, self.current_delay * self.multiplier)
        return delay

    def reset(self):
        self.current_delay = self.initial_delay


class ReconnectCommandGate:
    """Require a fresh zero command before motion after a reconnect."""

    def __init__(self):
        self.has_connected = False
        self.locked = False

    def connected(self):
        self.locked = self.has_connected
        self.has_connected = True

    def allow(self, linear_x, angular_z):
        if not self.locked:
            return True
        if linear_x == 0.0 and angular_z == 0.0:
            self.locked = False
            return True
        return False


class DryRunTcpTransport:
    def __init__(self, host, port, timeout=1.0, socket_factory=socket.create_connection):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.socket_factory = socket_factory
        self.socket = None
        self.session_id = None

    def connect(self):
        self.close()
        connection = self.socket_factory((self.host, self.port), timeout=self.timeout)
        connection.settimeout(self.timeout)
        try:
            session_id = decode_ready(self._readline(connection))
        except (ValueError, ConnectionError):
            connection.close()
            raise ConnectionError("unexpected dry-run receiver greeting")
        self.socket = connection
        self.session_id = session_id
        return session_id

    def send(self, sequence, linear_mm_s, angular_mrad_s):
        if self.socket is None:
            raise ConnectionError("dry-run transport is disconnected")
        self.socket.sendall(
            encode_command(self.session_id, sequence, linear_mm_s, angular_mrad_s)
        )
        ack_session, ack_sequence, state = decode_ack(self._readline(self.socket))
        if ack_session != self.session_id or ack_sequence != sequence:
            raise ConnectionError("dry-run ACK session or sequence mismatch")
        return state

    def close(self):
        connection, self.socket = self.socket, None
        self.session_id = None
        if connection is not None:
            try:
                connection.close()
            except OSError:
                pass

    @staticmethod
    def _readline(connection):
        data = bytearray()
        while len(data) < 160:
            chunk = connection.recv(1)
            if not chunk:
                raise ConnectionError("dry-run receiver disconnected")
            data.extend(chunk)
            if chunk == b"\n":
                return bytes(data)
        raise ConnectionError("dry-run receiver line too long")
