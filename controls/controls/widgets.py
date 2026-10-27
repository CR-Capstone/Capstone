"""Small shared input widgets used by more than one page."""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QDoubleSpinBox, QHBoxLayout, QSlider, QWidget

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
