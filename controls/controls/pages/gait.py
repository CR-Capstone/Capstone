"""Stage 2 — gait. Continuous waveform on both motors for a droopy robot.

The output is a steady counter-rotating base with a periodic term added to
both motors in common mode, matching ``droopy.control.Command`` in the
simulator: one rotor's magnitude rises while the other's falls. The sine is
the default shape; triangle, square and sawtooth are there to see how much of
the robot's behaviour depends on the shape rather than the frequency.

The preview is the steady-state waveform with the spin-up ramp removed. The
scope underneath it is what actually went down the wire.
"""
from __future__ import annotations

from dataclasses import replace

from PySide6.QtCore import Qt
from PySide6.QtCore import QPointF
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QFormLayout, QGroupBox, QHBoxLayout, QLabel,
    QPushButton, QSizePolicy, QSplitter, QVBoxLayout, QWidget,
)

from .. import dshot, theme
from ..packets import PacketView
from ..scope import MotorReadout, Scope
from ..waveform import SHAPES, TWO_PI, Gait
from ..widgets import ValueSlider
from .base import ControlPage, reverse_warning

PREVIEW_CYCLES = 2.0
PREVIEW_POINTS = 400


class GaitPreview(QWidget):
    """Steady-state shape of the current gait over two cycles."""

    def __init__(self, gait: Gait) -> None:
        super().__init__()
        self.gait = gait
        self.setMinimumHeight(150)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor(theme.BACKGROUND))
        rect = self.rect().adjusted(46, 10, -10, -20)
        steady = replace(self.gait, ramp_s=0.0, enabled=True)
        period = steady.period() or 1.0
        span = period * PREVIEW_CYCLES

        painter.setFont(QFont("Menlo", 9))
        for level in (1.0, 0.5, 0.0, -0.5, -1.0):
            y = rect.center().y() - level * rect.height() / 2.0
            saturated = abs(level) == 1.0
            painter.setPen(
                QPen(QColor(theme.DANGER if saturated else theme.GRID),
                     1, Qt.PenStyle.DashLine if saturated else Qt.PenStyle.SolidLine)
            )
            painter.drawLine(rect.left(), int(y), rect.right(), int(y))
            painter.setPen(QPen(QColor("#5d718c")))
            painter.drawText(4, int(y) + 4, f"{level:+.1f}")

        series = [[], []]
        for step in range(PREVIEW_POINTS + 1):
            t = span * step / PREVIEW_POINTS
            values = steady.raw(t)
            x = rect.left() + rect.width() * step / PREVIEW_POINTS
            for index in (0, 1):
                y = rect.center().y() - max(-1.3, min(1.3, values[index])) * rect.height() / 2.0
                series[index].append(QPointF(x, y))
        for index, color in enumerate(theme.MOTOR_COLORS):
            painter.setPen(QPen(QColor(color), 1.8))
            painter.drawPolyline(QPolygonF(series[index]))

        painter.setPen(QPen(QColor("#5d718c")))
        painter.drawText(
            rect.left(), self.height() - 5,
            f"{PREVIEW_CYCLES:.0f} cycles · {span:.2f} s · steady state, ramp excluded",
        )


class GaitPage(ControlPage):
    title = "2 · Gait"
    stage = 2
    summary = "Continuous sine on both motors: spin base plus a common-mode wave."

    def __init__(self, engine, link) -> None:
        super().__init__(engine, link)
        self.gait = Gait()

        layout = QHBoxLayout(self)
        left = QVBoxLayout()
        layout.addLayout(left, 2)
        left.addWidget(self._waveform_box())
        left.addWidget(self._safety_box())
        left.addStretch(1)

        right = QVBoxLayout()
        layout.addLayout(right, 4)

        wave_panel = QWidget()
        wave_layout = QVBoxLayout(wave_panel)
        wave_layout.setContentsMargins(0, 0, 0, 0)
        wave_layout.addWidget(QLabel("Commanded waveform", objectName="section"))
        self.preview = GaitPreview(self.gait)
        wave_layout.addWidget(self.preview, 1)

        scope_panel = QWidget()
        scope_layout = QVBoxLayout(scope_panel)
        scope_layout.setContentsMargins(0, 0, 0, 0)
        scope_layout.addWidget(QLabel("Transmitted", objectName="section"))
        self.readouts = [MotorReadout(0), MotorReadout(1)]
        for widget in self.readouts:
            scope_layout.addWidget(widget)
        self.scope = Scope(window=4.0)
        scope_layout.addWidget(self.scope, 1)
        self.status = QLabel(objectName="mono")
        self.status.setWordWrap(True)
        scope_layout.addWidget(self.status)

        self.packets = PacketView()
        split = QSplitter(Qt.Orientation.Vertical)
        waves = QSplitter(Qt.Orientation.Horizontal)
        waves.addWidget(wave_panel)
        waves.addWidget(scope_panel)
        waves.setSizes([400, 400])
        split.addWidget(waves)
        split.addWidget(self.packets)
        split.setSizes([460, 360])
        right.addWidget(split)
        self._refresh()

    # ---- widgets -------------------------------------------------------

    def _waveform_box(self) -> QGroupBox:
        box = QGroupBox("Waveform")
        form = QFormLayout(box)

        self.shape = QComboBox()
        self.shape.addItems(SHAPES)
        self.shape.currentTextChanged.connect(self._refresh)

        self.spin = ValueSlider(0.0, 1.0, self.gait.spin, decimals=3, step=0.01)
        self.amplitude = ValueSlider(0.0, 1.0, self.gait.amplitude, decimals=3, step=0.01)
        self.frequency = ValueSlider(0.0, 30.0, self.gait.frequency, " Hz", decimals=2, step=0.1)
        self.bias = ValueSlider(-0.5, 0.5, self.gait.bias, decimals=3, step=0.01)
        self.phase = ValueSlider(0.0, 360.0, self.gait.phase_deg, "°", decimals=1, step=1.0)
        self.offset = ValueSlider(0.0, 360.0, self.gait.offset_deg, "°", decimals=1, step=1.0)
        self.counter = QCheckBox("Counter-rotate the base (motor 2 negative)")
        self.counter.setChecked(self.gait.counter_rotate)
        self.counter.toggled.connect(self._refresh)

        for slider in (self.spin, self.amplitude, self.frequency,
                       self.bias, self.phase, self.offset):
            slider.changed.connect(self._refresh)

        self.spin.setToolTip("Steady base magnitude. Both motors run at this, opposite in sign.")
        self.amplitude.setToolTip("Size of the periodic term added to both motors.")
        self.frequency.setToolTip("Cycles per second. For a melty-style robot this is the spin rate.")
        self.offset.setToolTip("Phase shift applied to motor 2 only. Leave at 0 to match the simulator.")

        form.addRow("Shape", self.shape)
        form.addRow("Spin base", self.spin)
        form.addRow("Amplitude", self.amplitude)
        form.addRow("Frequency", self.frequency)
        form.addRow("Bias", self.bias)
        form.addRow("Phase", self.phase)
        form.addRow("Motor 2 offset", self.offset)
        form.addRow(self.counter)
        return box

    def _safety_box(self) -> QGroupBox:
        box = QGroupBox("Spin-up")
        form = QFormLayout(box)
        self.ramp = ValueSlider(0.0, 10.0, self.gait.ramp_s, " s", decimals=1, step=0.1)
        self.ramp.changed.connect(self._refresh)
        self.ramp.setToolTip("Output scales from zero to full over this time after arming.")
        self.enabled = QCheckBox("Waveform enabled")
        self.enabled.setChecked(True)
        self.enabled.toggled.connect(self._refresh)
        reset = QPushButton("Restart cycle")
        reset.clicked.connect(self._restart)
        reset.setToolTip("Disarms, so the phase and the ramp both start over.")
        form.addRow("Ramp time", self.ramp)
        form.addRow(self.enabled)
        form.addRow(reset)
        return box

    # ---- behaviour -----------------------------------------------------

    def _refresh(self, *_args) -> None:
        """Push every control into the gait and update the derived text."""
        self.gait.shape_name = self.shape.currentText()
        self.gait.spin = self.spin.value()
        self.gait.amplitude = self.amplitude.value()
        self.gait.frequency = self.frequency.value()
        self.gait.bias = self.bias.value()
        self.gait.phase_deg = self.phase.value()
        self.gait.offset_deg = self.offset.value()
        self.gait.counter_rotate = self.counter.isChecked()
        self.gait.ramp_s = self.ramp.value()
        self.gait.enabled = self.enabled.isChecked()
        self.preview.update()

        peaks, lows = self._extremes()
        period = self.gait.period()
        clipping = max(peaks) > 1.0
        warning = reverse_warning(self.engine, (lows[0] < 0.0, lows[1] < 0.0))
        lines = [
            f"period {period:6.3f} s    peak |demand|  "
            f"M1 {peaks[0]:.3f}  M2 {peaks[1]:.3f}"
        ]
        if clipping:
            lines.append("CLIPPING — reduce spin base, amplitude or bias.")
        if warning:
            lines.append(warning)
        self.packets.burst_frames = max(
            20, min(2000, round((period or 1.0) * self.engine.config.rate_hz))
        )
        lines.append(self._traffic_note())
        self.status.setText("\n".join(lines))
        self.status.setStyleSheet(
            f"color:{theme.DANGER}" if clipping else
            (f"color:{theme.WARN}" if warning else "")
        )

    def _traffic_note(self) -> str:
        """What this waveform costs on a change-driven link.

        A moving sine crosses the change threshold on nearly every evaluation,
        so the send rate sits at the evaluation rate until the threshold is
        raised enough to quantise it.
        """
        config = self.engine.config
        span = dshot.HALF_SPAN if config.maps[0].three_d else dshot.FULL_SPAN
        # Peak slope of the commanded wave, in DShot counts per evaluation.
        per_tick = (
            self.gait.amplitude * span * TWO_PI * self.gait.frequency
            / max(1, config.rate_hz)
        )
        if per_tick <= 0.0:
            return (
                f"holding still: one line every {config.keepalive_s:.2f} s "
                f"(keepalive)"
            )
        duty = min(1.0, per_tick / max(1, config.change_counts))
        return (
            f"moving at up to {per_tick:.1f} counts per evaluation vs a "
            f"{config.change_counts}-count threshold: about "
            f"{duty * config.rate_hz:.0f} of {config.rate_hz} evaluations send"
        )

    def _extremes(self) -> tuple[list[float], list[float]]:
        """Largest magnitude and smallest signed value per motor over one cycle."""
        steady = replace(self.gait, ramp_s=0.0, enabled=True)
        period = steady.period() or 1.0
        peaks, lows = [0.0, 0.0], [0.0, 0.0]
        for step in range(180):
            values = steady.raw(period * step / 180.0)
            for index in (0, 1):
                peaks[index] = max(peaks[index], abs(values[index]))
                lows[index] = min(lows[index], values[index])
        return peaks, lows

    def _restart(self) -> None:
        self.engine.disarm("cycle restart")
        self.scope.clear()
        self.packets.clear()

    def sample(self, t: float) -> tuple[float, float]:
        return self.gait.sample(t)

    def on_frame(self, frame) -> None:
        for index, readout in enumerate(self.readouts):
            readout.set_state(frame.demand[index], frame.values[index])
        self.scope.add(frame)
        self.packets.add(frame)

    def on_activated(self) -> None:
        self.scope.clear()
        self.packets.clear()
        self._refresh()
