"""Pure decoder for the fixed, Drive-free Roomba diagnostic query."""

import struct


ENCODER_QUERY = bytes([149, 2, 43, 44])

CHARGING_STATES = {
    0: "not_charging",
    1: "reconditioning_charging",
    2: "full_charging",
    3: "trickle_charging",
    4: "waiting",
    5: "charging_fault",
}

OI_MODES = {0: "off", 1: "passive", 2: "safe", 3: "full"}


def decode_diagnostic_response(response):
    """Decode packets 21, 22, 25, 26, 34, 35, and 7 in that order."""
    if not isinstance(response, (bytes, bytearray)):
        raise TypeError("response must be bytes")
    if len(response) != 10:
        raise ValueError("diagnostic response must be exactly 10 bytes")

    charging_state = response[0]
    charge_mah = (response[3] << 8) | response[4]
    capacity_mah = (response[5] << 8) | response[6]
    charging_sources = response[7]
    oi_mode = response[8]
    if charging_state not in CHARGING_STATES:
        raise ValueError("invalid charging state")
    if oi_mode not in OI_MODES:
        raise ValueError("invalid OI mode")
    if charging_sources > 3:
        raise ValueError("invalid charging sources bit field")

    wheel_bits = response[9]
    if wheel_bits > 0x0F:
        raise ValueError("invalid bumps and wheel drops bit field")
    return {
        "charging_state": charging_state,
        "charging_state_name": CHARGING_STATES[charging_state],
        "voltage_mv": (response[1] << 8) | response[2],
        "charge_mah": charge_mah,
        "capacity_mah": capacity_mah,
        "charge_percent": charge_mah * 100.0 / capacity_mah if capacity_mah else None,
        "charging_source_internal": bool(charging_sources & 0x01),
        "charging_source_home_base": bool(charging_sources & 0x02),
        "oi_mode": oi_mode,
        "oi_mode_name": OI_MODES[oi_mode],
        "wheel_drop_right": bool(wheel_bits & 0x04),
        "wheel_drop_left": bool(wheel_bits & 0x08),
        "bump_right": bool(wheel_bits & 0x01),
        "bump_left": bool(wheel_bits & 0x02),
    }


def decode_encoder_response(response):
    """Decode packet 43 (left) and packet 44 (right) without issuing I/O."""
    if not isinstance(response, (bytes, bytearray)):
        raise TypeError("encoder response must be bytes")
    if len(response) != 4:
        raise ValueError("encoder response must be exactly 4 bytes")
    return struct.unpack(">HH", response)
