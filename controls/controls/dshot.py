"""DShot frame construction and signed-demand to throttle mapping.

A DShot frame is 16 bits::

    bits 15..5  11-bit value
    bit  4      telemetry request
    bits 3..0   checksum

The value is 0 for disarmed, 1..47 for ESC commands and 48..2047 for throttle.
The checksum is the XOR of the three nibbles of ``value << 1 | telemetry``;
bidirectional DShot inverts it so the ESC can reuse the line for RPM telemetry.

Nothing here touches hardware. The 16-bit integers these functions return are
what travels over the serial link; the firmware bit-bangs them.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

VALUE_BITS = 11
VALUE_MAX = 2047
DISARMED = 0
MIN_THROTTLE = 48
# 3-D split: 48..1047 is one direction, 1048..2047 the other, each a full ramp.
REVERSE_MIN, REVERSE_MAX = 48, 1047
FORWARD_MIN, FORWARD_MAX = 1048, 2047
HALF_SPAN = REVERSE_MAX - REVERSE_MIN  # 999
FULL_SPAN = VALUE_MAX - MIN_THROTTLE   # 1999


class Command(IntEnum):
    """Standard DShot special commands. Most want ~10 repeats to take."""

    MOTOR_STOP = 0
    BEEP1 = 1
    BEEP2 = 2
    BEEP3 = 3
    BEEP4 = 4
    BEEP5 = 5
    ESC_INFO = 6
    SPIN_DIRECTION_1 = 7
    SPIN_DIRECTION_2 = 8
    MODE_3D_OFF = 9
    MODE_3D_ON = 10
    SETTINGS_REQUEST = 11
    SAVE_SETTINGS = 12
    EXTENDED_TELEMETRY_ENABLE = 13
    EXTENDED_TELEMETRY_DISABLE = 14
    SPIN_DIRECTION_NORMAL = 20
    SPIN_DIRECTION_REVERSED = 21
    LED0_ON = 22
    LED1_ON = 23
    LED2_ON = 24
    LED3_ON = 25
    LED0_OFF = 26
    LED1_OFF = 27
    LED2_OFF = 28
    LED3_OFF = 29


def checksum(payload: int, bidirectional: bool = False) -> int:
    """Four-bit XOR of the payload nibbles, inverted for bidirectional DShot."""
    value = payload & 0xFFF
    result = 0
    for _ in range(3):
        result ^= value
        value >>= 4
    result &= 0xF
    return (~result & 0xF) if bidirectional else result


def frame(value: int, telemetry: bool = False, bidirectional: bool = False) -> int:
    """Pack an 11-bit value into the 16-bit frame sent over the link."""
    value = int(value)
    if not 0 <= value <= VALUE_MAX:
        raise ValueError(f"DShot value {value} outside 0..{VALUE_MAX}")
    payload = (value << 1) | (1 if telemetry else 0)
    return (payload << 4) | checksum(payload, bidirectional)


def decode(word: int) -> tuple[int, bool, bool]:
    """Return ``(value, telemetry, checksum_ok)`` for a non-inverted frame."""
    word = int(word) & 0xFFFF
    payload = word >> 4
    return payload >> 1, bool(payload & 1), checksum(payload) == (word & 0xF)


@dataclass
class MotorMap:
    """Per-motor conversion from signed demand in [-1, 1] to a DShot value.

    ``three_d`` selects the split 3-D range, which is what a counter-rotating
    pair needs when both ESCs are wired identically. Without it the magnitude
    is used and the sign is dropped, because a unidirectional ESC has no way
    to express reverse.
    """

    invert: bool = False
    three_d: bool = False
    deadband: float = 0.02
    limit: float = 1.0

    def value(self, demand: float) -> int:
        """Clamp, scale and quantise ``demand`` into a DShot value."""
        demand = float(demand)
        if demand != demand:  # NaN never reaches an ESC.
            return DISARMED
        demand = max(-1.0, min(1.0, demand))
        if self.invert:
            demand = -demand
        limit = max(0.0, min(1.0, self.limit))
        demand *= limit
        if abs(demand) <= self.deadband:
            return DISARMED
        if not self.three_d:
            return MIN_THROTTLE + round(abs(demand) * FULL_SPAN)
        if demand >= 0.0:
            return FORWARD_MIN + round(demand * HALF_SPAN)
        return REVERSE_MIN + round(-demand * HALF_SPAN)

    def demand_of(self, value: int) -> float:
        """Inverse of :meth:`value`, for read-back and display."""
        value = int(value)
        if value < MIN_THROTTLE:
            return 0.0
        if not self.three_d:
            result = (value - MIN_THROTTLE) / FULL_SPAN
        elif value >= FORWARD_MIN:
            result = (value - FORWARD_MIN) / HALF_SPAN
        else:
            result = -(value - REVERSE_MIN) / HALF_SPAN
        return -result if self.invert else result


def clipped(demand: float, mapping: MotorMap) -> bool:
    """True when the mapping had to clamp, so the UI can show saturation."""
    return abs(float(demand)) > max(0.0, min(1.0, mapping.limit))
