"""UART-free velocity logging protocol shared by ROS 2 and Pico W."""

PROTOCOL_VERSION = 2
MAX_LINEAR_MM_S = 500
MAX_ANGULAR_MRAD_S = 5000


class ProtocolError(ValueError):
    pass


def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _frame(fields):
    payload = "|".join(str(value) for value in fields).encode("ascii")
    return payload + b"|%04X\n" % crc16_ccitt(payload)


def _validate_session(session_id):
    if not isinstance(session_id, int) or isinstance(session_id, bool) or session_id < 0:
        raise ProtocolError("session_id must be a non-negative integer")


def encode_ready(session_id):
    _validate_session(session_id)
    return _frame(("READY", PROTOCOL_VERSION, session_id))


def decode_ready(line):
    parts = _decode(line, b"READY", 4)
    try:
        session_id = int(parts[2])
    except ValueError as exc:
        raise ProtocolError("invalid session encoding") from exc
    _validate_session(session_id)
    return session_id


def encode_command(session_id, sequence, linear_mm_s, angular_mrad_s):
    _validate_session(session_id)
    if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 0:
        raise ProtocolError("sequence must be a non-negative integer")
    linear_mm_s = int(linear_mm_s)
    angular_mrad_s = int(angular_mrad_s)
    if abs(linear_mm_s) > MAX_LINEAR_MM_S:
        raise ProtocolError("linear velocity out of range")
    if abs(angular_mrad_s) > MAX_ANGULAR_MRAD_S:
        raise ProtocolError("angular velocity out of range")
    return _frame(("CMD", PROTOCOL_VERSION, session_id, sequence, linear_mm_s, angular_mrad_s))


def _decode(line, expected_kind, field_count):
    if not isinstance(line, (bytes, bytearray)) or len(line) > 160:
        raise ProtocolError("invalid frame type or length")
    parts = bytes(line).strip().split(b"|")
    if len(parts) != field_count:
        raise ProtocolError("invalid field count")
    payload = b"|".join(parts[:-1])
    try:
        supplied_crc = int(parts[-1], 16)
    except ValueError as exc:
        raise ProtocolError("invalid checksum encoding") from exc
    if crc16_ccitt(payload) != supplied_crc:
        raise ProtocolError("checksum mismatch")
    if parts[0] != expected_kind or int(parts[1]) != PROTOCOL_VERSION:
        raise ProtocolError("unsupported frame kind or version")
    return parts


def decode_command(line):
    parts = _decode(line, b"CMD", 7)
    try:
        session_id, sequence, linear_mm_s, angular_mrad_s = map(int, parts[2:6])
    except ValueError as exc:
        raise ProtocolError("non-integer command field") from exc
    encode_command(session_id, sequence, linear_mm_s, angular_mrad_s)
    return session_id, sequence, linear_mm_s, angular_mrad_s


def encode_ack(session_id, sequence, state):
    _validate_session(session_id)
    if state not in ("ACTIVE", "STOPPED"):
        raise ProtocolError("invalid acknowledgement state")
    return _frame(("ACK", PROTOCOL_VERSION, session_id, int(sequence), state))


def decode_ack(line):
    parts = _decode(line, b"ACK", 6)
    session_id = int(parts[2])
    sequence = int(parts[3])
    state = parts[4].decode("ascii")
    _validate_session(session_id)
    if sequence < 0 or state not in ("ACTIVE", "STOPPED"):
        raise ProtocolError("invalid acknowledgement")
    return session_id, sequence, state


class DryRunReceiverState:
    """Fail-safe state machine. It never imports or writes a UART."""

    def __init__(self, session_id, timeout_ms=500):
        if timeout_ms <= 0:
            raise ValueError("timeout_ms must be positive")
        self.timeout_ms = timeout_ms
        _validate_session(session_id)
        self.session_id = session_id
        self.last_sequence = -1
        self.last_received_ms = None
        self.linear_mm_s = 0
        self.angular_mrad_s = 0
        self.state = "STOPPED"
        self.reason = "startup"

    def stop(self, reason):
        self.linear_mm_s = 0
        self.angular_mrad_s = 0
        self.state = "STOPPED"
        self.reason = reason

    def accept(self, line, now_ms):
        try:
            session_id, sequence, linear_mm_s, angular_mrad_s = decode_command(line)
            if session_id != self.session_id:
                raise ProtocolError("session mismatch")
            if sequence <= self.last_sequence:
                raise ProtocolError("sequence is not increasing")
        except (ProtocolError, ValueError):
            self.stop("invalid_frame")
            raise
        self.last_sequence = sequence
        self.last_received_ms = now_ms
        self.linear_mm_s = linear_mm_s
        self.angular_mrad_s = angular_mrad_s
        self.state = "STOPPED" if (linear_mm_s, angular_mrad_s) == (0, 0) else "ACTIVE"
        self.reason = "stop_command" if self.state == "STOPPED" else "command"
        return sequence, self.state

    def poll(self, now_ms):
        if (
            self.last_received_ms is not None
            and self.state == "ACTIVE"
            and now_ms - self.last_received_ms > self.timeout_ms
        ):
            self.stop("receive_timeout")
            return True
        return False

    def disconnect(self):
        self.stop("communication_lost")
