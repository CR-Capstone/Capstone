"""The one place control frames are produced and sent.

Exactly one page drives the output at a time. The active page supplies a
``sample(t) -> (demand1, demand2)`` callable, where each demand is signed and
normalised to [-1, 1]; the engine turns that into DShot values through the
per-motor :class:`~controls.dshot.MotorMap` and frames them.

The link is change-driven, not a stream. The receiver ESP32 repeats the last
value it got in hardware until a new one arrives, so the engine sends a line
only when the values actually moved, and otherwise resends the held line once
per keepalive interval to prove the link is alive. The evaluation rate is
therefore a ceiling on the send rate, not the send rate.

The ESP32s only relay values; this engine owns the ESC protocol. Arming is a
sequence, not a switch (``disarmed -> arming -> armed``):

1. If the link went quiet for longer than the receiver's failsafe timeout, the
   receiver is locked out and ignores everything for ``lockout_s``. Keep
   sending 0 until that has certainly passed.
2. Value 0 on both motors for ``arm_zero_s`` (1.6 s), so the ESCs arm.
3. Idle (3-D neutral, 1048) for ``arm_neutral_s`` (0.4 s).
4. Hand the output to the page, with its clock starting at zero.

Rules the pages cannot override:

* Disarmed means value 0 on both motors, sent immediately and repeated.
* Armed zero demand is the motor map's idle (3-D neutral), never value 0, and
  a transition to or from idle always goes out whatever the change threshold.
* If the app itself goes quiet past the failsafe timeout while armed, the
  receiver has cut the ESCs, so the engine disarms rather than resuming.
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

DEFAULT_RATE_HZ = 100
RATE_RANGE = (10, 1000)
# remote-dshot's ramp_test.py sends every 50 ms.
DEFAULT_KEEPALIVE_S = 0.05
KEEPALIVE_RANGE = (0.01, 10.0)
# receiver.c: FAILSAFE_TIMEOUT_MS and LOCKOUT_MS.
DEFAULT_HOLD_TIMEOUT_S = 0.5
HOLD_TIMEOUT_RANGE = (0.1, 60.0)
DEFAULT_LOCKOUT_S = 10.0
# ramp_test.py: DSHOT_ARM_CMD0_DURATION_MS and DSHOT_ARM_CENTER_DURATION_MS.
DEFAULT_ARM_ZERO_S = 1.6
DEFAULT_ARM_NEUTRAL_S = 0.4
# Below this ratio of timeout to keepalive, one lost packet trips the failsafe.
SAFE_KEEPALIVE_RATIO = 3.0
DISARM_REPEATS = 8

CHANGE = "change"
KEEPALIVE = "keepalive"
COMMAND = "command"
DISARM = "disarm"
ARM = "arm"

DISARMED = "disarmed"
ARMING = "arming"
ARMED = "armed"


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
    # What the receiver firmware does on silence. The app cannot change these;
    # it uses them to check the keepalive margin and to wait out a lockout.
    hold_timeout_s: float = DEFAULT_HOLD_TIMEOUT_S
    lockout_s: float = DEFAULT_LOCKOUT_S
    arm_zero_s: float = DEFAULT_ARM_ZERO_S
    arm_neutral_s: float = DEFAULT_ARM_NEUTRAL_S
    change_counts: int = 1
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
        self._state = DISARMED
        self._origin = time.perf_counter()
        self._arm_started = self._origin
        self._pending: list[_Pending] = []
        self._tick_times: list[float] = []
        self._send_times: list[float] = []
        self._last_values: tuple[int, int] | None = None
        self._last_send_at = -1e9
        # Only lines that reached the port count towards the receiver's
        # failsafe. Nothing has been sent yet, so assume it may be locked out.
        self._last_wire_at = -1e9
        self._alive_since: float | None = None
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
        """True from the moment the operator arms, through the sequence."""
        return self._state != DISARMED

    @property
    def state(self) -> str:
        return self._state

    def arm(self) -> bool:
        """Start the arming sequence, if a source is plugged in and the port is open."""
        if self._state != DISARMED:
            return True
        if self._source is None or not self.link.is_open:
            return False
        self._pending.clear()
        self._arm_started = time.perf_counter()
        self._state = ARMING
        self.armed_changed.emit(True, f"arming · {self._source_name}")
        return True

    def disarm(self, reason: str = "operator") -> None:
        """Drop to value 0 and push disarm frames out immediately."""
        was_armed = self._state != DISARMED
        self._state = DISARMED
        self._pending.clear()
        for _ in range(DISARM_REPEATS):
            self._emit(
                self.elapsed, (0.0, 0.0), (dshot.DISARMED, dshot.DISARMED), DISARM
            )
        if was_armed:
            self.armed_changed.emit(False, reason)

    def receiver_ready_in(self) -> float:
        """Seconds until a lockout the receiver may be in has certainly ended."""
        return max(0.0, self._receiver_ready_at() - time.perf_counter())

    def _receiver_ready_at(self) -> float:
        """When the receiver is certain to forward again.

        A lockout starts when the receiver notices a gap, which is no later
        than the first line after it, and lasts ``lockout_s``. Lines that have
        flowed since then without a gap keep it forwarding.
        """
        if self._alive_since is None:
            return time.perf_counter() + self.config.lockout_s
        return self._alive_since + self.config.lockout_s

    def arming_status(self) -> str:
        """Where the arming sequence is, for the banner. Empty unless arming."""
        if self._state != ARMING:
            return ""
        phase, left = self._arming_phase(time.perf_counter())
        return f"{phase} {left:.1f} s"

    def _arming_phase(self, now: float) -> tuple[str, float]:
        """``(phase, seconds left in it)``; phase is lockout, zero, neutral or done."""
        config = self.config
        zero_from = max(self._arm_started, self._receiver_ready_at())
        if now < zero_from:
            return "lockout", zero_from - now
        zero_until = zero_from + config.arm_zero_s
        if now < zero_until:
            return "zero", zero_until - now
        neutral_until = zero_until + config.arm_neutral_s
        if now < neutral_until:
            return "neutral", neutral_until - now
        return "done", 0.0

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
        self, value: int, repeats: int = 3, motors: tuple[bool, bool] = (True, True)
    ) -> None:
        """Queue a DShot command (1..47). Ignored while armed; ESCs want zero throttle.

        The receiver repeats each value in hardware until the next line, so one
        line already reaches the ESC as a continuous run of command frames. The
        repeats are there to survive a dropped ESP-NOW packet.
        """
        value = int(value)
        if not 0 <= value < dshot.MIN_THROTTLE:
            raise ValueError(f"{value} is not a DShot command value")
        if self._state != DISARMED:
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

        # The receiver has already cut the ESCs and locked out; resuming
        # mid-gait would hand them throttle without an arming sequence.
        # While arming, a gap only pushes the sequence back to the lockout wait.
        if (self._state == ARMED
                and now - self._last_wire_at > self.config.hold_timeout_s):
            self.disarm("link went quiet, receiver failsafe")
            return

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

        if self._state == DISARMED or self._source is None:
            self._offer(t, (0.0, 0.0), (dshot.DISARMED, dshot.DISARMED))
            return

        if self._state == ARMING:
            phase, _left = self._arming_phase(now)
            if phase in ("lockout", "zero"):
                self._offer(t, (0.0, 0.0), (dshot.DISARMED, dshot.DISARMED), ARM)
                return
            if phase == "neutral":
                idle = tuple(m.idle for m in self.config.maps)
                self._offer(t, (0.0, 0.0), idle, ARM)
                return
            self._state = ARMED
            self._origin = now
            t = 0.0
            self.armed_changed.emit(True, self._source_name)

        try:
            demand = self._source(t)
            pair = (float(demand[0]), float(demand[1]))
        except Exception:
            self.disarm("source error")
            return
        values = tuple(m.value(d) for m, d in zip(self.config.maps, pair))
        self._offer(t, pair, values)

    def _offer(
        self,
        t: float,
        demand: tuple[float, float],
        values: tuple[int, int],
        why: str = CHANGE,
    ) -> None:
        """Send if the values moved, or if the keepalive is due. Else hold."""
        reason = self._reason_to_send(values)
        if reason is None:
            self.frames_held += 1
            return
        self._emit(t, demand, values, why if reason == CHANGE else reason)

    def _reason_to_send(self, values: tuple[int, int]) -> str | None:
        previous = self._last_values
        if previous is None:
            return CHANGE
        # Starting or stopping a motor is never suppressed by the threshold.
        if any(self._still(v, m) != self._still(p, m)
               for v, p, m in zip(values, previous, self.config.maps)):
            return CHANGE
        threshold = max(1, int(self.config.change_counts))
        if any(abs(v - p) >= threshold for v, p in zip(values, previous)):
            return CHANGE
        # The epsilon keeps float rounding from slipping a keepalive a whole tick.
        if time.perf_counter() - self._last_send_at >= self.config.keepalive_s - 1e-9:
            return KEEPALIVE
        return None

    @staticmethod
    def _still(value: int, mapping: dshot.MotorMap) -> bool:
        return value == dshot.DISARMED or value == mapping.idle

    def _emit(
        self,
        t: float,
        demand: tuple[float, float],
        values: tuple[int, int],
        reason: str,
    ) -> None:
        # What the receiver builds from each value and clocks out to the ESC.
        words = tuple(dshot.frame(v) for v in values)
        line = protocol.encode(values[0], values[1])
        sent = self.link.write(line)
        now = time.perf_counter()
        if sent:
            if now - self._last_wire_at > self.config.hold_timeout_s:
                self._alive_since = now
            self._last_wire_at = now
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
