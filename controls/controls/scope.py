"""Rolling display of what was actually transmitted.

The scope is fed from :attr:`controls.engine.OutputEngine.frame_sent`, so it
shows the sent demand, not the UI's intent. If the two disagree, the mapping
or a clamp is the reason.
"""
from __future__ import annotations

from collections import deque

from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QSizePolicy, QWidget

from . import dshot, theme


class Scope(QWidget):
    """Last ``window`` seconds of both motors' normalised demand."""

    def __init__(self, window: float = 4.0, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.window = float(window)
        self.samples: deque[tuple[float, float, float]] = deque(maxlen=20000)
        self.last = None
        self.setMinimumHeight(190)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def add(self, frame) -> None:
        """Append one :class:`~controls.engine.TxFrame`."""
        self.samples.append((frame.t, frame.demand[0], frame.demand[1]))
        self.last = frame
        cutoff = frame.t - self.window
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()
        self.update()

    def clear(self) -> None:
        self.samples.clear()
        self.last = None
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = self.rect().adjusted(46, 10, -10, -20)
        painter.fillRect(self.rect(), QColor(theme.BACKGROUND))

        painter.setFont(QFont("Menlo", 9))
        for level in (1.0, 0.5, 0.0, -0.5, -1.0):
            y = rect.center().y() - level * rect.height() / 2.0
            painter.setPen(QPen(QColor(theme.GRID), 2 if level == 0.0 else 1))
            painter.drawLine(rect.left(), int(y), rect.right(), int(y))
            painter.setPen(QPen(QColor("#5d718c")))
            painter.drawText(4, int(y) + 4, f"{level:+.1f}")

        if len(self.samples) < 2:
            painter.setPen(QPen(QColor("#5d718c")))
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, "no frames yet")
            return

        newest = self.samples[-1][0]
        span = max(self.window, 1e-6)

        def to_point(t: float, value: float) -> QPointF:
            x = rect.right() - (newest - t) / span * rect.width()
            y = rect.center().y() - max(-1.0, min(1.0, value)) * rect.height() / 2.0
            return QPointF(x, y)

        for index, color in enumerate(theme.MOTOR_COLORS):
            polygon = QPolygonF([to_point(s[0], s[1 + index]) for s in self.samples])
            painter.setPen(QPen(QColor(color), 1.6))
            painter.drawPolyline(polygon)

        if self.last is not None:
            painter.setFont(QFont("Menlo", 10))
            for index, color in enumerate(theme.MOTOR_COLORS):
                value = self.last.values[index]
                word = self.last.words[index]
                flag = "  CLIP" if self.last.clipped[index] else ""
                painter.setPen(QPen(QColor(color)))
                painter.drawText(
                    rect.left() + 6,
                    rect.top() + 14 + index * 15,
                    f"M{index + 1}  {self.last.demand[index]:+.3f}  "
                    f"dshot {value:4d}  ESC 0x{word:04X}{flag}",
                )
        painter.setPen(QPen(QColor("#5d718c")))
        painter.setFont(QFont("Menlo", 9))
        painter.drawText(rect.left(), self.height() - 5, f"{self.window:.1f} s window")


class MotorReadout(QWidget):
    """Compact per-motor bar: signed demand plus the DShot value it became."""

    def __init__(self, index: int, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.index = index
        self.demand = 0.0
        self.value = dshot.DISARMED
        self.setMinimumHeight(38)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    def set_state(self, demand: float, value: int) -> None:
        self.demand, self.value = float(demand), int(value)
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = self.rect().adjusted(0, 0, -1, -1)
        painter.fillRect(rect, QColor("#0c131d"))
        middle = rect.center().x()
        color = QColor(theme.MOTOR_COLORS[self.index])
        width = max(-1.0, min(1.0, self.demand)) * (rect.width() / 2.0 - 2)
        painter.fillRect(
            int(min(middle, middle + width)), rect.top() + 2,
            int(abs(width)), rect.height() - 4, color,
        )
        painter.setPen(QPen(QColor(theme.GRID)))
        painter.drawLine(middle, rect.top(), middle, rect.bottom())
        painter.setPen(QPen(QColor(theme.TEXT)))
        painter.setFont(QFont("Menlo", 10))
        painter.drawText(
            rect.adjusted(8, 0, -8, 0),
            Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft,
            f"M{self.index + 1}  {self.demand:+6.3f}",
        )
        painter.drawText(
            rect.adjusted(8, 0, -8, 0),
            Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight,
            f"dshot {self.value:4d}",
        )
