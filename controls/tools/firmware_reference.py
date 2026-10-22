"""Reference model of the remote-dshot receiver, for checking the app offline.

The real path is: this app -> serial -> sender ESP32 -> ESP-NOW -> receiver
ESP32 -> DShot -> ESC. The sender forwards ``"<a> <b>\\n"`` lines with two bare
DShot values in 0..2047. The receiver (``main/receiver.c``) is a three-state
machine:

* IDLE — no output. The first valid packet starts output.
* ACTIVE — every packet replaces the value being repeated to the ESCs. A gap
  longer than the failsafe timeout (500 ms) stops output.
* LOCKOUT — no output, every packet ignored for 10 s, then back to IDLE.

:class:`Receiver` mirrors that, so the app's timing can be checked against a
capture or a live port without the radio link::

    python tools/firmware_reference.py capture.txt
    python tools/firmware_reference.py /dev/ttyACM0
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from controls import dshot, protocol  # noqa: E402

DEFAULT_FAILSAFE_S = 0.5
DEFAULT_LOCKOUT_S = 10.0

IDLE = "IDLE"
ACTIVE = "ACTIVE"
LOCKOUT = "LOCKOUT"


def describe(values: tuple[int, int]) -> str:
    """One row for a pair of values, named the way an ESC reads them."""
    fields = []
    for index, value in enumerate(values):
        kind = (
            "stop/arm" if value == dshot.DISARMED
            else f"command {value}" if value < dshot.MIN_THROTTLE
            else "neutral" if value == dshot.NEUTRAL_3D
            else f"throttle {value}"
        )
        fields.append(f"M{index + 1} {value:4d} {kind:<13}")
    return "  ".join(fields)


class Receiver:
    """The receiver's state machine, driven by arrival times."""

    def __init__(
        self,
        failsafe_s: float = DEFAULT_FAILSAFE_S,
        lockout_s: float = DEFAULT_LOCKOUT_S,
    ) -> None:
        self.failsafe_s = float(failsafe_s)
        self.lockout_s = float(lockout_s)
        self.state = IDLE
        self.output: tuple[int, int] | None = None
        self.last_rx: float | None = None
        self.lockout_start = 0.0
        self.accepted = 0
        self.rejected = 0
        self.ignored = 0
        self.failsafe_trips = 0

    def receive(self, line: str, now: float) -> str:
        """Take one line arriving at ``now``. Returns the row to print."""
        tripped = self.tick(now)
        prefix = f"{tripped}\n" if tripped else ""
        try:
            values = protocol.decode(line)
        except ValueError as exc:
            self.rejected += 1
            return f"{prefix}REJECT  {line.strip()!r}  ({exc})"
        if self.state == LOCKOUT:
            self.ignored += 1
            left = self.lockout_s - (now - self.lockout_start)
            return f"{prefix}IGNORE  {describe(values)}  (lockout, {left:.1f} s left)"
        gap = "" if self.last_rx is None else f"  +{now - self.last_rx:.3f}s"
        started = "  OUTPUT STARTED" if self.state == IDLE else ""
        self.state = ACTIVE
        self.output = values
        self.last_rx = now
        self.accepted += 1
        return f"{prefix}FORWARD {describe(values)}{gap}{started}"

    def tick(self, now: float) -> str | None:
        """Advance the clock. Returns a row when the state changes."""
        if self.state == ACTIVE and now - self.last_rx > self.failsafe_s:
            self.state = LOCKOUT
            self.output = None
            self.lockout_start = now
            self.failsafe_trips += 1
            return (
                f"FAILSAFE  gap of {now - self.last_rx:.3f} s — output stopped, "
                f"locked out for {self.lockout_s:.0f} s"
            )
        if self.state == LOCKOUT and now - self.lockout_start >= self.lockout_s:
            self.state = IDLE
            return "IDLE      lockout over, waiting for packets"
        return None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", help="serial device or a text file of lines")
    parser.add_argument("--failsafe", type=float, default=DEFAULT_FAILSAFE_S)
    parser.add_argument("--lockout", type=float, default=DEFAULT_LOCKOUT_S)
    parser.add_argument("--limit", type=int, default=0, help="stop after N lines")
    args = parser.parse_args(argv)

    receiver = Receiver(args.failsafe, args.lockout)
    path = Path(args.source)

    if not (path.exists() and path.is_file()):
        import serial

        port = serial.Serial(timeout=0.05)
        port.port = args.source
        port.dtr = False
        port.rts = False
        port.open()
        count = 0
        try:
            while not args.limit or count < args.limit:
                raw = port.readline().decode("utf-8", "replace")
                now = time.monotonic()
                if raw.strip():
                    print(receiver.receive(raw, now))
                    count += 1
                else:
                    row = receiver.tick(now)
                    if row:
                        print(row)
        except KeyboardInterrupt:
            pass
    else:
        # A capture has no timing, so lines are treated as arriving 50 ms apart.
        for count, line in enumerate(path.read_text().splitlines(), 1):
            if line.strip():
                print(receiver.receive(line + "\n", count * 0.05))
            if args.limit and count >= args.limit:
                break

    print(
        f"-- forwarded {receiver.accepted}  rejected {receiver.rejected}  "
        f"ignored {receiver.ignored}  failsafe trips {receiver.failsafe_trips}  "
        f"state {receiver.state}  output {receiver.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
