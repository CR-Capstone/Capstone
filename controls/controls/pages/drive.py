"""Stage 3 — drive. Walk the droopy gait and steer it with a bias.

Same gait as stage 2, driven from the keyboard. Holding W engages the walking
amplitude; A and D add a small constant bias to both rotors, which is the
steering knob the droopy simulator's control study found (``droopy/docs/
control-study.md``): about 0.04 of full speed moved the 9 lb model's travel
heading by roughly 20 degrees over established cycles, with no loss of speed.

There is no heading dial. The gait is open loop, so its phase is only a time
offset and cannot point the robot anywhere; the direction it walks comes from
floor contact and has to be found on the floor.

Amplitude and bias slew at a fixed rate rather than stepping, because a step
on a heavy rotor is a current spike and a likely desync.
"""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox, QFormLayout, QGroupBox, QHBoxLayout, QLabel, QVBoxLayout,
)

from .. import theme
from ..scope import MotorReadout, Scope
from ..waveform import Gait
from ..widgets import ValueSlider
from .base import ControlPage, reverse_warning, zero_crossing_warning

WALK_KEYS = {Qt.Key.Key_W, Qt.Key.Key_Up}
# Positive bias turned the simulated robot's travel clockwise.
STEER_KEYS = {
    Qt.Key.Key_A: -1.0,
    Qt.Key.Key_Left: -1.0,
    Qt.Key.Key_D: 1.0,
    Qt.Key.Key_Right: 1.0,
}


def slew(current: float, target: float, rate: float, dt: float) -> float:
    """Move ``current`` toward ``target`` by at most ``rate * dt``."""
    step = max(0.0, rate) * max(0.0, dt)
    return current + max(-step, min(step, target - current))


class DrivePage(ControlPage):
    title = "3 · Drive"
    stage = 3
    summary = "Walk the gait with W, steer it with A / D bias."

    def __init__(self, engine, link) -> None:
        super().__init__(engine, link)
        self.gait = Gait(amplitude=0.0)
        self._held: set = set()
        self._amplitude = 0.0
        self._bias = 0.0
        self._last_t: float | None = None
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        layout = QHBoxLayout(self)
        left = QVBoxLayout()
        layout.addLayout(left, 2)
        hint = QLabel(
            "Hold W / ↑ to walk. Hold A / ← or D / → to steer.\n"
            "Release W to stop walking; the counter-spin keeps running "
            "until you disarm.\n"
            "Steering sign comes from the simulator. Check it on the floor.",
            objectName="mono",
        )
        hint.setWordWrap(True)
        left.addWidget(hint)
        left.addWidget(self._drive_box())
        left.addStretch(1)

        right = QVBoxLayout()
        layout.addLayout(right, 3)
        right.addWidget(QLabel("Transmitted", objectName="section"))
        self.readouts = [MotorReadout(0), MotorReadout(1)]
        for widget in self.readouts:
            right.addWidget(widget)
        self.scope = Scope(window=6.0)
        right.addWidget(self.scope, 1)
        self.status = QLabel(objectName="mono")
        self.status.setWordWrap(True)
        right.addWidget(self.status)
        self._refresh()

    def _drive_box(self) -> QGroupBox:
        box = QGroupBox("Drive")
        form = QFormLayout(box)
        self.spin = ValueSlider(0.0, 1.0, 0.49, decimals=3, step=0.01)
        self.frequency = ValueSlider(0.0, 5.0, 0.83, " Hz", decimals=2, step=0.05)
        self.walk = ValueSlider(0.0, 0.6, 0.40, decimals=3, step=0.01)
        self.steer = ValueSlider(0.0, 0.15, 0.04, decimals=3, step=0.005)
        self.slew_rate = ValueSlider(0.05, 2.0, 0.5, " /s", decimals=2, step=0.05)
        self.ramp = ValueSlider(0.0, 10.0, 3.0, " s", decimals=1, step=0.1)
        self.hold = QCheckBox("Keep walking without W held")
        self.hold.setToolTip("Leave off for dead-man behaviour.")
        self.invert_steer = QCheckBox("Invert steering")
        for widget in (self.spin, self.frequency, self.walk, self.steer,
                       self.slew_rate, self.ramp):
            widget.changed.connect(self._refresh)
        for widget in (self.hold, self.invert_steer):
            widget.toggled.connect(self._refresh)
        self.spin.setToolTip("Counter-rotating base. The sim gait uses 0.49 of max speed.")
        self.frequency.setToolTip("Gait cycles per second. The 9 lb sim gait is 0.83 Hz.")
        self.walk.setToolTip("Common-mode sine amplitude while walking. The sim uses 0.40.")
        self.steer.setToolTip("Bias added to both rotors while A or D is held.")
        self.slew_rate.setToolTip("How fast walk amplitude and steering bias may change.")
        form.addRow("Spin base", self.spin)
        form.addRow("Gait frequency", self.frequency)
        form.addRow("Walk amplitude", self.walk)
        form.addRow("Steer bias", self.steer)
        form.addRow("Slew limit", self.slew_rate)
        form.addRow("Spin-up ramp", self.ramp)
        form.addRow(self.hold)
        form.addRow(self.invert_steer)
        return box

    # ---- behaviour -----------------------------------------------------

    def targets(self) -> tuple[float, float]:
        """Walk amplitude and steering bias the held keys ask for."""
        walking = bool(self._held & WALK_KEYS) or self.hold.isChecked()
        direction = sum(STEER_KEYS.get(key, 0.0) for key in self._held)
        direction = max(-1.0, min(1.0, direction))
        if self.invert_steer.isChecked():
            direction = -direction
        return (
            self.walk.value() if walking else 0.0,
            direction * self.steer.value(),
        )

    def _refresh(self, *_args) -> None:
        self.gait.spin = self.spin.value()
        self.gait.frequency = self.frequency.value()
        self.gait.ramp_s = self.ramp.value()
        amplitude, bias = self.targets()
        peak = self.gait.spin + self.walk.value() + self.steer.value()
        floor = self.gait.spin - self.walk.value() - self.steer.value()
        warnings = [
            reverse_warning(self.engine, (False, True)),
            zero_crossing_warning((floor < 0.0, floor < 0.0)),
        ]
        text = (
            f"walk {amplitude:.3f} (now {self._amplitude:.3f})   "
            f"bias {bias:+.3f} (now {self._bias:+.3f})   "
            f"worst-case |demand| {floor:.3f} to {peak:.3f}"
        )
        self.status.setText("\n".join([text] + [w for w in warnings if w]))
        self.status.setStyleSheet(
            f"color:{theme.DANGER}" if peak > 1.0 else
            (f"color:{theme.WARN}" if any(warnings) else "")
        )

    def keyPressEvent(self, event) -> None:
        if event.isAutoRepeat():
            return
        key = Qt.Key(event.key())
        if key in WALK_KEYS or key in STEER_KEYS:
            self._held.add(key)
            self._refresh()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event) -> None:
        if event.isAutoRepeat():
            return
        key = Qt.Key(event.key())
        if key in WALK_KEYS or key in STEER_KEYS:
            self._held.discard(key)
            self._refresh()
            return
        super().keyReleaseEvent(event)

    def sample(self, t: float) -> tuple[float, float]:
        if self._last_t is None or t < self._last_t:  # armed again
            self._amplitude = self._bias = 0.0
            self._last_t = t
        dt, self._last_t = t - self._last_t, t
        amplitude, bias = self.targets()
        rate = self.slew_rate.value()
        self._amplitude = slew(self._amplitude, amplitude, rate, dt)
        self._bias = slew(self._bias, bias, rate, dt)
        self.gait.amplitude = self._amplitude
        self.gait.bias = self._bias
        return self.gait.sample(t)

    def on_frame(self, frame) -> None:
        for index, readout in enumerate(self.readouts):
            readout.set_state(frame.demand[index], frame.values[index])
        self.scope.add(frame)
        self._refresh()

    def on_activated(self) -> None:
        self._held.clear()
        self._last_t = None
        self.scope.clear()
        self._refresh()
        self.setFocus()

    def on_deactivated(self) -> None:
        self._held.clear()
        self._refresh()
