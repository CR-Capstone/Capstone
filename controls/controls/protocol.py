"""The serial line contract shared with the sender ESP32 (remote-dshot).

One text line per control update::

    "<motor 1 value> <motor 2 value>\\n"        e.g.  b"1548 548\\n"

Each number is the bare 11-bit DShot value, 0..2047, in decimal: 0 disarms or
arms, 1..47 are ESC commands, 48..2047 throttle. The receiver ESP32 builds the
16-bit frame and checksum itself. The sender rejects anything else (a third
field, a sign, hex, a value over 2047) with a ``Bad line`` log on the same port.

Keep every change to the wire format in this module. Nothing else in the app
builds or reads a line.
"""
from __future__ import annotations

TERMINATOR = "\n"
ENCODING = "ascii"
VALUE_MAX = 2047


def encode(value1: int, value2: int) -> bytes:
    """Serialise one control update. Raises on anything outside 0..2047."""
    words = []
    for value in (value1, value2):
        value = int(value)
        if not 0 <= value <= VALUE_MAX:
            raise ValueError(f"DShot value {value} outside 0..{VALUE_MAX}")
        words.append(str(value))
    return (" ".join(words) + TERMINATOR).encode(ENCODING)


def decode(line: str) -> tuple[int, int]:
    """Parse one update back out of a line, as strictly as the sender does."""
    text = line.rstrip("\r\n")
    parts = text.split(" ")
    if len(parts) != 2 or not all(part.isdigit() for part in parts):
        raise ValueError(f"expected two unsigned decimal fields: {line!r}")
    values = tuple(int(part) for part in parts)
    for value in values:
        if value > VALUE_MAX:
            raise ValueError(f"DShot value {value} outside 0..{VALUE_MAX}")
    return values[0], values[1]
