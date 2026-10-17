"""The one place control frames are produced and sent.

Exactly one page drives the output at a time. The active page supplies a
``sample(t) -> (demand1, demand2)`` callable, where each demand is signed and
normalised to [-1, 1]; the engine turns that into DShot values through the
per-motor :class:`~controls.dshot.MotorMap` and frames them.

The link is change-driven, not a stream. The firmware holds the last packet it
received until a new one arrives or its own timeout expires, so the engine
sends a frame only when the values actually moved, and otherwise resends the
held frame once per keepalive interval to prove the link is alive. The
evaluation rate is therefore a ceiling on the send rate, not the send rate.

Two rules the pages cannot override:

* Disarmed means value 0 on both motors, sent immediately and repeated, and a
  transition to or from zero always goes out whatever the change threshold is.
* The waveform clock comes from :func:`time.perf_counter`, not from the timer
  tick count, so timer jitter shifts when a sample is taken but never distorts
  the wave it carries.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable

from PySide6.QtCore import QObject, Qt, QTimer, Signal

from . import dshot, protocol
from .link import SerialLink

Source = Callable[[float], tuple[float, float]]

DEFAULT_RATE_HZ = 200
RATE_RANGE = (10, 1000)
DEFAULT_KEEPALIVE_S = 0.5
KEEPALIVE_RANGE = (0.02, 10.0)
DEFAULT_HOLD_TIMEOUT_S = 2.0
HOLD_TIMEOUT_RANGE = (0.1, 60.0)
# Below this ratio of timeout to keepalive, one lost packet trips the failsafe.
SAFE_KEEPALIVE_RATIO = 3.0
DISARM_REPEATS = 8

CHANGE = "change"
KEEPALIVE = "keepalive"
COMMAND = "command"
DISARM = "disarm"


@dataclass(frozen=True)
class TxFrame:
    """One transmitted control frame, as the scope and packet view see it."""

    t: float
    demand: tuple[float, float]
    values: tuple[int, int]
    words: tuple[int, int]
    clipped: tuple[bool, bool]
    sent: bool
    line: bytes
    reason: str = CHANGE


@dataclass
class _Pending:
    """A DShot special command waiting to be repeated onto one or both motors."""

    value: int
    repeats: int
    motors: tuple[bool, bool] = (True, True)


@dataclass
class EngineConfig:
    """Wire, timing and mapping settings, owned by the link page."""

    rate_hz: int = DEFAULT_RATE_HZ
    keepalive_s: float = DEFAULT_KEEPALIVE_S
    # What the firmware is configured to do, not something this app enforces.
    # It is here so the UI can check the keepalive has margin against it.
    hold_timeout_s: float = DEFAULT_HOLD_TIMEOUT_S
    change_counts: int = 1
    bidirectional: bool = False
    hex_mode: bool = False
    telemetry: bool = False
    # 3-D by default: the gait pages counter-rotate, and a plain throttle range
    # has no way to express a negative demand.
    maps: list[dshot.MotorMap] = field(
        default_factory=lambda: [
            dshot.MotorMap(three_d=True), dshot.MotorMap(three_d=True)
        ]
    )


class OutputEngine(QObject):
    """Change-driven transmitter with a keepalive and one pluggable source."""

    frame_sent = Signal(object)          # TxFrame
    armed_changed = Signal(bool, str)    # armed, reason
    source_changed = Signal(str)         # page title or ""

    def __init__(self, link: SerialLink, config: EngineConfig | None = None) -> None:
        super().__init__()
        self.link = link
        self.config = config or EngineConfig()
        self._source: Source | None = None
        self._source_name = ""
        self._armed = False
        self._origin = time.perf_counter()
        self._pending: list[_Pending] = []
        self._tick_times: list[float] = []
        self._send_times: list[float] = []
        self._last_values: tuple[int, int] | None = None
        self._last_send_at = -1e9
        self.frames_held = 0
        self.last_frame: TxFrame | None = None
        self._timer = QTimer(self)
        self._timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._timer.timeout.connect(self._tick)
        self.apply_rate(self.config.rate_hz)
        link.closed.connect(lambda reason: self.disarm(f"link {reason}"))
        link.failed.connect(lambda reason: self.disarm("link error"))
        link.opened.connect(lambda _device: self._resend_now())

    # ---- configuration -------------------------------------------------

    def apply_rate(self, rate_hz: int) -> None:
        """Set how often the source is evaluated, which caps the send rate."""
        low, high = RATE_RANGE
        rate = max(low, min(high, int(rate_hz)))
        self.config.rate_hz = rate
        self._timer.start(max(1, round(1000.0 / rate)))

    def start(self) -> None:
        self._timer.start(max(1, round(1000.0 / self.config.rate_hz)))

    def stop(self) -> None:
        self._timer.stop()

    # ---- arming --------------------------------------------------------

    @property
    def armed(self) -> bool:
        return self._armed

    def arm(self) -> bool:
        """Arm, if a source is plugged in and the port is open."""
        if self._armed:
            return True
        if self._source is None or not self.link.is_open:
            return False
        self._origin = time.perf_counter()
        self._armed = True
        self.armed_changed.emit(True, self._source_name)
        return True

    def disarm(self, reason: str = "operator") -> None:
        """Drop to value 0 and push disarm frames out immediately."""
        was_armed = self._armed
        self._armed = False
        self._pending.clear()
        for _ in range(DISARM_REPEATS):
            self._emit(
                self.elapsed, (0.0, 0.0), (dshot.DISARMED, dshot.DISARMED), DISARM
            )
        if was_armed:
            self.armed_changed.emit(False, reason)

    # ---- source --------------------------------------------------------

    def set_source(self, source: Source | None, name: str = "") -> None:
        """Swap the active waveform. Always disarms: a new page is a new robot."""
        self.disarm("page change")
        self._source = source
        self._source_name = name
        self.source_changed.emit(name)

    @property
    def elapsed(self) -> float:
        return time.perf_counter() - self._origin

    # ---- special commands ----------------------------------------------

    def queue_command(
        self, value: int, repeats: int = 10, motors: tuple[bool, bool] = (True, True)
    ) -> None:
        """Queue a DShot command (1..47). Ignored while armed; ESCs want zero throttle."""
        value = int(value)
        if not 0 <= value < dshot.MIN_THROTTLE:
            raise ValueError(f"{value} is not a DShot command value")
        if self._armed:
            return
        self._pending.append(_Pending(value, max(1, int(repeats)), motors))

    # ---- rates ---------------------------------------------------------

    def measured_tick_rate(self) -> float:
        """How often the source is being evaluated."""
        return _rate_of(self._tick_times)

    def measured_send_rate(self) -> float:
        """How often a frame actually reaches the wire."""
        return _rate_of(self._send_times)

    def _resend_now(self) -> None:
        """Forget what the firmware is holding, so the next tick sends."""
        self._last_values = None
        self._last_send_at = -1e9

    # ---- transmit ------------------------------------------------------

    def _tick(self) -> None:
        now = time.perf_counter()
        self._tick_times.append(now)
        if len(self._tick_times) > 128:
            del self._tick_times[:-128]
        t = now - self._origin

        if self._pending:
            job = self._pending[0]
            job.repeats -= 1
            if job.repeats <= 0:
                self._pending.pop(0)
            values = tuple(
                job.value if use else dshot.DISARMED for use in job.motors
            )
            self._emit(t, (0.0, 0.0), values, COMMAND)
            return

        if not self._armed or self._source is None:
            self._offer(t, (0.0, 0.0), (dshot.DISARMED, dshot.DISARMED))
            return

        try:
            demand = self._source(t)
            pair = (float(demand[0]), float(demand[1]))
        except Exception:
            self.disarm("source error")
            return
        values = tuple(m.value(d) for m, d in zip(self.config.maps, pair))
        self._offer(t, pair, values)

    def _offer(
        self, t: float, demand: tuple[float, float], values: tuple[int, int]
    ) -> None:
        """Send if the values moved, or if the keepalive is due. Else hold."""
        reason = self._reason_to_send(values)
        if reason is None:
            self.frames_held += 1
            return
        self._emit(t, demand, values, reason)

    def _reason_to_send(self, values: tuple[int, int]) -> str | None:
        previous = self._last_values
        if previous is None:
            return CHANGE
        # Starting or stopping a motor is never suppressed by the threshold.
        if any((v == dshot.DISARMED) != (p == dshot.DISARMED)
               for v, p in zip(values, previous)):
            return CHANGE
        threshold = max(1, int(self.config.change_counts))
        if any(abs(v - p) >= threshold for v, p in zip(values, previous)):
            return CHANGE
        if time.perf_counter() - self._last_send_at >= self.config.keepalive_s:
            return KEEPALIVE
        return None

    def _emit(
        self,
        t: float,
        demand: tuple[float, float],
        values: tuple[int, int],
        reason: str,
    ) -> None:
        words = tuple(
            dshot.frame(v, self.config.telemetry, self.config.bidirectional)
            for v in values
        )
        line = protocol.encode(words[0], words[1], self.config.hex_mode)
        sent = self.link.write(line)
        now = time.perf_counter()
        self._last_values = values
        self._last_send_at = now
        self._send_times.append(now)
        if len(self._send_times) > 128:
            del self._send_times[:-128]
        frame = TxFrame(
            t=t,
            demand=demand,
            values=values,
            words=words,
            clipped=tuple(
                dshot.clipped(d, m) for d, m in zip(demand, self.config.maps)
            ),
            sent=sent,
            line=line,
            reason=reason,
        )
        self.last_frame = frame
        self.frame_sent.emit(frame)


def keepalive_margin(config: EngineConfig) -> str:
    """Empty when the keepalive has room, otherwise why it does not.

    The firmware trips its failsafe after ``hold_timeout_s`` of silence. A
    keepalive close to that leaves no room for a dropped packet, so the rule
    of thumb is at least three keepalives inside the timeout.
    """
    if config.keepalive_s <= 0.0:
        return "keepalive is zero"
    allowed = config.hold_timeout_s / SAFE_KEEPALIVE_RATIO
    if config.keepalive_s <= allowed:
        return ""
    misses = max(0, int(config.hold_timeout_s // config.keepalive_s) - 1)
    return (
        f"keepalive {config.keepalive_s:.2f} s against a "
        f"{config.hold_timeout_s:.2f} s hold: only {misses} lost packet"
        f"{'' if misses == 1 else 's'} before the firmware fails safe. "
        f"Keep it at or under {allowed:.2f} s."
    )


def _rate_of(times: list[float]) -> float:
    """Events per second over the recorded window, or 0 when too few or stale."""
    if len(times) < 2:
        return 0.0
    span = times[-1] - times[0]
    if span <= 0.0:
        return 0.0
    # A window that ended a while ago is reported as idle, not as a stale rate.
    if time.perf_counter() - times[-1] > max(1.0, 2.0 * span / (len(times) - 1)):
        return 0.0
    return (len(times) - 1) / span
