"""Small shared input widgets used by more than one page."""
from __future__ import annotations

import math

from PySide6.QtCore import QPointF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QDoubleSpinBox, QHBoxLayout, QSizePolicy, QSlider, QWidget,
)

from . import theme

SLIDER_STEPS = 1000


class ValueSlider(QWidget):
    """A slider and a spin box over one value, kept in sync."""

    changed = Signal(float)

    def __init__(
        self,
        low: float,
        high: float,
        value: float,
        suffix: str = "",
        decimals: int = 2,
        step: float = 0.01,
    ) -> None:
        super().__init__()
        self.low, self.high = float(low), float(high)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, SLIDER_STEPS)
        self.box = QDoubleSpinBox()
        self.box.setRange(self.low, self.high)
        self.box.setDecimals(decimals)
        self.box.setSingleStep(step)
        self.box.setSuffix(suffix)
        self.box.setFixedWidth(96)
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.box)
        self._guard = False
        self.slider.valueChanged.connect(self._from_slider)
        self.box.valueChanged.connect(self._from_box)
        self.setValue(value)

    def value(self) -> float:
        return self.box.value()

    def setValue(self, value: float) -> None:
        value = max(self.low, min(self.high, float(value)))
        self._guard = True
        self.box.setValue(value)
        span = self.high - self.low or 1.0
        self.slider.setValue(round((value - self.low) / span * SLIDER_STEPS))
        self._guard = False
        self.changed.emit(value)

    def _from_slider(self, raw: int) -> None:
        if self._guard:
            return
        value = self.low + (self.high - self.low) * raw / SLIDER_STEPS
        self._guard = True
        self.box.setValue(value)
        self._guard = False
        self.changed.emit(value)

    def _from_box(self, value: float) -> None:
        if self._guard:
            return
        span = self.high - self.low or 1.0
        self._guard = True
        self.slider.setValue(round((value - self.low) / span * SLIDER_STEPS))
        self._guard = False
        self.changed.emit(value)


class HeadingDial(QWidget):
    """Click-and-drag compass. Emits the requested heading in degrees.

    Heading is the gait's phase: it chooses where in each rotation the common
    perturbation peaks. Whether 0 degrees points where the robot actually goes
    depends on wiring and the real contact behaviour, so treat the number as a
    repeatable handle, not a calibrated direction.
    """

    changed = Signal(float)

    def __init__(self, heading: float = 0.0) -> None:
        super().__init__()
        self._heading = float(heading)
        self.setMinimumSize(170, 170)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

    def heading(self) -> float:
        return self._heading

    def setHeading(self, degrees: float) -> None:
        self._heading = float(degrees) % 360.0
        self.update()
        self.changed.emit(self._heading)

    def mousePressEvent(self, event) -> None:
        self._track(event)

    def mouseMoveEvent(self, event) -> None:
        if event.buttons() & Qt.MouseButton.LeftButton:
            self._track(event)

    def _track(self, event) -> None:
        center = QPointF(self.width() / 2.0, self.height() / 2.0)
        delta = event.position() - center
        self.setHeading(math.degrees(math.atan2(delta.x(), -delta.y())))

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor(theme.BACKGROUND))
        center = QPointF(self.width() / 2.0, self.height() / 2.0)
        radius = min(self.width(), self.height()) / 2.0 - 14

        painter.setPen(QPen(QColor(theme.GRID), 1))
        painter.drawEllipse(center, radius, radius)
        painter.setFont(QFont("Menlo", 9))
        for degrees in range(0, 360, 30):
            angle = math.radians(degrees)
            outer = QPointF(
                center.x() + math.sin(angle) * radius,
                center.y() - math.cos(angle) * radius,
            )
            inner = QPointF(
                center.x() + math.sin(angle) * radius * 0.88,
                center.y() - math.cos(angle) * radius * 0.88,
            )
            painter.setPen(QPen(QColor(theme.GRID), 1))
            painter.drawLine(inner, outer)

        angle = math.radians(self._heading)
        tip = QPointF(
            center.x() + math.sin(angle) * radius * 0.82,
            center.y() - math.cos(angle) * radius * 0.82,
        )
        painter.setPen(QPen(QColor(theme.MOTOR_COLORS[0]), 3))
        painter.drawLine(center, tip)
        painter.setPen(QPen(QColor(theme.TEXT)))
        painter.drawText(
            self.rect().adjusted(0, 0, 0, -2),
            Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignBottom,
            f"{self._heading:5.1f}°",
        )
