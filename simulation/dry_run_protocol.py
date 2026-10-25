"""Host codec for the UART-free Pico W velocity logging protocol."""

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


def decode_ready(line):
    parts = _decode(line, b"READY", 4)
    try:
        session_id = int(parts[2])
    except ValueError as exc:
        raise ProtocolError("invalid session encoding") from exc
    _validate_session(session_id)
    return session_id


def decode_ack(line):
    parts = _decode(line, b"ACK", 6)
    try:
        session_id = int(parts[2])
        sequence = int(parts[3])
        state = parts[4].decode("ascii")
    except (ValueError, UnicodeError) as exc:
        raise ProtocolError("invalid ACK encoding") from exc
    _validate_session(session_id)
    if sequence < 0 or state not in ("ACTIVE", "STOPPED"):
        raise ProtocolError("invalid acknowledgement")
    return session_id, sequence, state
