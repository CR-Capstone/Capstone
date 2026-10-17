"""Reference receiver for the control line, as the firmware has to implement it.

The contract has two halves. The laptop sends a packet whenever the values
change and resends the held packet on a keepalive. The firmware holds the last
packet it accepted and drives the ESCs from it until either a new packet
arrives or it has heard nothing for the hold timeout, at which point it fails
safe to DShot value 0.

:class:`Hold` is that receiver. Run it here against a port or a capture so the
behaviour can be checked without hardware::

    python tools/firmware_reference.py /dev/cu.usbmodem1101
    python tools/firmware_reference.py capture.txt --hold 2.0

A real firmware does the same four things: split the line, verify both
checksums, reject the frame if either fails, and fail safe on silence.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from controls import dshot, protocol  # noqa: E402

DEFAULT_HOLD_S = 2.0


def describe(words: tuple[int, int]) -> str:
    """One row for a pair of frames, decoded the way an ESC reads them."""
    fields = []
    for index, word in enumerate(words):
        value, telemetry, ok = dshot.decode(word)
        kind = (
            "disarmed" if value == dshot.DISARMED
            else f"command {value}" if value < dshot.MIN_THROTTLE
            else f"throttle {value}"
        )
        fields.append(
            f"M{index + 1} 0x{word:04X} {kind:<14}"
            f" telem={int(telemetry)} crc={'ok' if ok else 'BAD'}"
        )
    return "  ".join(fields)


class Hold:
    """The last accepted packet, and the timeout that gives it up."""

    def __init__(self, timeout_s: float = DEFAULT_HOLD_S) -> None:
        self.timeout_s = float(timeout_s)
        self.words: tuple[int, int] = (0, 0)
        self.last_accepted_at: float | None = None
        self.failsafe = True
        self.accepted = 0
        self.rejected = 0
        self.failsafe_trips = 0

    def receive(self, line: str, now: float) -> str:
        """Take one line. Returns the row to print, held state included."""
        try:
            words = protocol.decode(line)
        except ValueError as exc:
            self.rejected += 1
            return f"REJECT  {line.strip()!r}  ({exc})"
        for word in words:
            if not dshot.decode(word)[2]:
                self.rejected += 1
                return f"REJECT  {describe(words)}  (checksum)"
        gap = "" if self.last_accepted_at is None else (
            f"  +{now - self.last_accepted_at:.3f}s"
        )
        resumed = "  RESUMED" if self.failsafe and self.accepted else ""
        self.words = words
        self.last_accepted_at = now
        self.failsafe = False
        self.accepted += 1
        return f"HOLD    {describe(words)}{gap}{resumed}"

    def tick(self, now: float) -> str | None:
        """Check for silence. Returns a row when the failsafe trips."""
        if self.failsafe or self.last_accepted_at is None:
            return None
        if now - self.last_accepted_at < self.timeout_s:
            return None
        self.failsafe = True
        self.failsafe_trips += 1
        self.words = (0, 0)
        return (
            f"FAILSAFE  no packet for {self.timeout_s:.2f} s — "
            f"outputs forced to value 0"
        )

    @property
    def values(self) -> tuple[int, int]:
        """The DShot values the ESCs are being driven with right now."""
        return tuple(dshot.decode(word)[0] for word in self.words)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", help="serial device or a text file of lines")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--hold", type=float, default=DEFAULT_HOLD_S,
                        help="hold timeout in seconds")
    parser.add_argument("--limit", type=int, default=0, help="stop after N lines")
    args = parser.parse_args(argv)

    hold = Hold(args.hold)
    path = Path(args.source)
    live = not (path.exists() and path.is_file())

    if live:
        import serial

        port = serial.Serial(args.source, args.baud, timeout=0.1)
        count = 0
        try:
            while not args.limit or count < args.limit:
                raw = port.readline().decode("utf-8", "replace")
                now = time.monotonic()
                if raw.strip():
                    print(hold.receive(raw, now))
                    count += 1
                tripped = hold.tick(now)
                if tripped:
                    print(tripped)
        except KeyboardInterrupt:
            pass
    else:
        # A capture has no timing, so every line is treated as arriving in turn.
        for count, line in enumerate(path.read_text().splitlines(), 1):
            if line.strip():
                print(hold.receive(line, float(count)))
            if args.limit and count >= args.limit:
                break

    print(
        f"-- accepted {hold.accepted}  rejected {hold.rejected}  "
        f"failsafe trips {hold.failsafe_trips}  holding {hold.values}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
