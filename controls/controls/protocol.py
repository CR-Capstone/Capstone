"""The serial line contract shared with the firmware.

One text line per control frame::

    "<motor 1 frame> <motor 2 frame>\\n"        e.g.  b"49541 17544\\n"

Each number is the complete 16-bit DShot frame from :mod:`controls.dshot`,
unsigned, decimal by default. Hexadecimal is accepted on both sides for
hand-typed bench work; the firmware distinguishes the two by the ``0x`` prefix.

Keep every change to the wire format in this module. Nothing else in the app
builds or reads a line.
"""
from __future__ import annotations

TERMINATOR = "\n"
ENCODING = "ascii"


def encode(frame1: int, frame2: int, hex_mode: bool = False) -> bytes:
    """Serialise one control frame. Raises on anything that is not 16-bit."""
    words = []
    for word in (frame1, frame2):
        word = int(word)
        if not 0 <= word <= 0xFFFF:
            raise ValueError(f"frame {word} outside 0..65535")
        words.append(f"0x{word:04X}" if hex_mode else str(word))
    return (" ".join(words) + TERMINATOR).encode(ENCODING)


def decode(line: str) -> tuple[int, int]:
    """Parse one control frame back out of a line, for tests and the console."""
    parts = line.strip().split()
    if len(parts) != 2:
        raise ValueError(f"expected two fields, got {len(parts)}: {line!r}")
    words = []
    for part in parts:
        word = int(part, 16) if part.lower().startswith("0x") else int(part, 10)
        if not 0 <= word <= 0xFFFF:
            raise ValueError(f"frame {word} outside 0..65535")
        words.append(word)
    return words[0], words[1]
