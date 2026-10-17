"""Stage 3 — drive. Spin continuously and steer by moving the wave's phase.

Same gait as stage 2, driven from a heading instead of raw numbers: the dial
or the keyboard sets the phase, and the translate control sets how much
amplitude that heading gets. Spin rate and base stay under direct control
because they decide whether the robot stays upright at all.

Heading is a phase, not a surveyed direction. Zero degrees is wherever the
wiring and contact behaviour put it; find the offset on the floor and keep it.
"""
from __future__ import annotations

import math

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox, QFormLayout, QGroupBox, QHBoxLayout, QLabel, QVBoxLayout,
)

from .. import theme
from ..scope import MotorReadout, Scope
from ..waveform import Gait
from ..widgets import HeadingDial, ValueSlider
from .base import ControlPage, reverse_warning

KEY_VECTORS = {
    Qt.Key.Key_W: (0.0, 1.0),
    Qt.Key.Key_Up: (0.0, 1.0),
    Qt.Key.Key_S: (0.0, -1.0),
    Qt.Key.Key_Down: (0.0, -1.0),
    Qt.Key.Key_A: (-1.0, 0.0),
    Qt.Key.Key_Left: (-1.0, 0.0),
    Qt.Key.Key_D: (1.0, 0.0),
    Qt.Key.Key_Right: (1.0, 0.0),
}


class DrivePage(ControlPage):
    title = "3 · Drive"
    stage = 3
    summary = "Spin plus steered translation from the dial or WASD."

    def __init__(self, engine, link) -> None:
        super().__init__(engine, link)
        self.gait = Gait(amplitude=0.0)
        self._held: set = set()
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        layout = QHBoxLayout(self)
        left = QVBoxLayout()
        layout.addLayout(left, 2)

        dial_row = QHBoxLayout()
        self.dial = HeadingDial()
        self.dial.changed.connect(lambda _v: self._refresh())
        dial_row.addWidget(self.dial)
        hint = QLabel(
            "Drag the dial or hold W A S D / arrows.\n"
            "Release every key to stop translating.\n"
            "The spin keeps running until you disarm.",
            objectName="mono",
        )
        hint.setWordWrap(True)
        dial_row.addWidget(hint, 1)
        left.addLayout(dial_row)
        left.addWidget(self._drive_box())
        left.addStretch(1)

        right = QVBoxLayout()
        layout.addLayout(right, 3)
        right.addWidget(QLabel("Transmitted", objectName="section"))
        self.readouts = [MotorReadout(0), MotorReadout(1)]
        for widget in self.readouts:
            right.addWidget(widget)
        self.scope = Scope(window=4.0)
        right.addWidget(self.scope, 1)
        self.status = QLabel(objectName="mono")
        self.status.setWordWrap(True)
        right.addWidget(self.status)
        self._refresh()

    def _drive_box(self) -> QGroupBox:
        box = QGroupBox("Drive")
        form = QFormLayout(box)
        self.spin = ValueSlider(0.0, 1.0, 0.35, decimals=3, step=0.01)
        self.frequency = ValueSlider(0.0, 30.0, 1.0, " Hz", decimals=2, step=0.1)
        self.translate = ValueSlider(0.0, 0.6, 0.20, decimals=3, step=0.01)
        self.ramp = ValueSlider(0.0, 10.0, 1.5, " s", decimals=1, step=0.1)
        self.hold = QCheckBox("Hold translation without a key held")
        self.hold.setToolTip("Dial-only driving. Leave off for dead-man behaviour.")
        for widget in (self.spin, self.frequency, self.translate, self.ramp):
            widget.changed.connect(self._refresh)
        self.hold.toggled.connect(self._refresh)
        self.translate.setToolTip("Amplitude applied at the commanded heading.")
        form.addRow("Spin base", self.spin)
        form.addRow("Spin rate", self.frequency)
        form.addRow("Translate", self.translate)
        form.addRow("Ramp time", self.ramp)
        form.addRow(self.hold)
        return box

    # ---- behaviour -----------------------------------------------------

    def _key_vector(self) -> tuple[float, float] | None:
        """Combined unit vector of the held keys, or None when nothing is held."""
        x = y = 0.0
        for key in self._held:
            dx, dy = KEY_VECTORS[key]
            x += dx
            y += dy
        magnitude = math.hypot(x, y)
        if magnitude < 1e-9:
            return None
        return x / magnitude, y / magnitude

    def _refresh(self, *_args) -> None:
        vector = self._key_vector()
        if vector is not None:
            self.dial.blockSignals(True)
            self.dial.setHeading(math.degrees(math.atan2(vector[0], vector[1])))
            self.dial.blockSignals(False)
        engaged = vector is not None or self.hold.isChecked()
        self.gait.spin = self.spin.value()
        self.gait.frequency = self.frequency.value()
        self.gait.amplitude = self.translate.value() if engaged else 0.0
        self.gait.phase_deg = self.dial.heading()
        self.gait.ramp_s = self.ramp.value()
        peak = self.gait.spin + self.gait.amplitude
        warning = reverse_warning(self.engine, (False, peak > 0.0))
        text = (
            f"heading {self.dial.heading():5.1f}°   amplitude {self.gait.amplitude:.3f}   "
            f"peak {peak:.3f}   {'TRANSLATING' if engaged else 'spin only'}"
        )
        self.status.setText(f"{text}\n{warning}" if warning else text)
        self.status.setStyleSheet(
            f"color:{theme.DANGER}" if peak > 1.0 else
            (f"color:{theme.WARN}" if warning else "")
        )

    def keyPressEvent(self, event) -> None:
        if event.isAutoRepeat():
            return
        key = Qt.Key(event.key())
        if key in KEY_VECTORS:
            self._held.add(key)
            self._refresh()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event) -> None:
        if event.isAutoRepeat():
            return
        key = Qt.Key(event.key())
        if key in KEY_VECTORS:
            self._held.discard(key)
            self._refresh()
            return
        super().keyReleaseEvent(event)

    def sample(self, t: float) -> tuple[float, float]:
        return self.gait.sample(t)

    def on_frame(self, frame) -> None:
        for index, readout in enumerate(self.readouts):
            readout.set_state(frame.demand[index], frame.values[index])
        self.scope.add(frame)

    def on_activated(self) -> None:
        self._held.clear()
        self.scope.clear()
        self._refresh()
        self.setFocus()

    def on_deactivated(self) -> None:
        self._held.clear()
        self._refresh()
