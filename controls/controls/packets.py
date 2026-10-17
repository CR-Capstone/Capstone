"""Literal view of the frames leaving the app.

The scope shows demand over time; this shows the bytes. For the most recent
frame it breaks each motor down the way the ESC reads it — payload, value,
telemetry bit, checksum — and prints the exact line that went out. Below that
is a decimated log, because a 200 Hz stream is not readable at full rate.

Only frames that actually went out appear here. Frames the engine held back
because nothing changed are counted in the status bar, not logged.
"""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QGuiApplication
from PySide6.QtWidgets import (
    QCheckBox, QHBoxLayout, QLabel, QPlainTextEdit, QPushButton, QSpinBox,
    QVBoxLayout, QWidget,
)

from . import dshot, theme

LOG_LINES = 2000
DETAIL_HZ = 20.0


def describe(value: int, word: int, demand: float, index: int) -> str:
    """Three lines showing how one motor's demand became one 16-bit frame."""
    decoded, telemetry, ok = dshot.decode(word)
    payload = word >> 4
    kind = (
        "disarmed" if decoded == dshot.DISARMED
        else f"command {decoded}" if decoded < dshot.MIN_THROTTLE
        else "throttle"
    )
    return (
        f"M{index + 1}   demand {demand:+.3f}   dshot {decoded:4d}  {kind}\n"
        f"     payload 0x{payload:03X}   value 0b{decoded:011b}   "
        f"telem {int(telemetry)}   crc 0x{word & 0xF:X} {'ok' if ok else 'BAD'}\n"
        f"     frame   0x{word:04X}   {word}"
    )


class PacketView(QWidget):
    """Last frame in full, plus a rate-limited log of the raw lines."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.burst_frames = 200
        self._last_log_t = -1e9
        self._last_detail_t = -1e9
        self._burst_left = 0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(QLabel("Packets on the wire", objectName="section"))

        self.detail = QLabel(objectName="mono")
        self.detail.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse
        )
        self.detail.setAlignment(
            Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft
        )
        self.detail.setStyleSheet(
            f"background:#0a1017; border:1px solid {theme.GRID}; padding:8px;"
        )
        self.detail.setText("no frames yet")
        layout.addWidget(self.detail)

        controls = QHBoxLayout()
        self.pause = QCheckBox("Freeze")
        self.pause.setToolTip("Stops updating this panel. Transmission continues.")
        self.rate = QSpinBox()
        self.rate.setRange(1, 200)
        self.rate.setValue(10)
        self.rate.setSuffix(" lines/s")
        self.rate.setToolTip("Ceiling on log lines. Frames the engine held are never logged.")
        burst = QPushButton("Capture burst")
        burst.setToolTip("Log every frame for one cycle, with no decimation.")
        burst.clicked.connect(self._start_burst)
        copy = QPushButton("Copy")
        copy.clicked.connect(self._copy)
        clear = QPushButton("Clear")
        clear.clicked.connect(self.clear)
        controls.addWidget(self.pause)
        controls.addWidget(self.rate)
        controls.addWidget(burst)
        controls.addWidget(copy)
        controls.addWidget(clear)
        controls.addStretch(1)
        layout.addLayout(controls)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(LOG_LINES)
        self.log.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        layout.addWidget(self.log, 1)
        self._header()

    # ---- feed ----------------------------------------------------------

    def add(self, frame) -> None:
        """Take one :class:`~controls.engine.TxFrame` from the engine."""
        if self.pause.isChecked():
            return
        if frame.t < self._last_log_t:  # the clock restarted on arming
            self._last_log_t = self._last_detail_t = -1e9
        if frame.t - self._last_detail_t >= 1.0 / DETAIL_HZ:
            self._last_detail_t = frame.t
            self._show(frame)
        bursting = self._burst_left > 0
        if bursting:
            self._burst_left -= 1
            if self._burst_left == 0:
                self.log.appendPlainText("--- burst end")
        elif frame.t - self._last_log_t < 1.0 / max(1, self.rate.value()):
            return
        self._last_log_t = frame.t
        self.log.appendPlainText(
            f"{frame.t:8.3f}  {frame.line!r:<22}"
            f"  M1 {frame.values[0]:4d}  M2 {frame.values[1]:4d}"
            f"  {frame.reason:<9}"
            f"{'' if frame.sent else '  NOT SENT'}"
        )

    def clear(self) -> None:
        self.log.clear()
        self._header()
        self.detail.setText("no frames yet")
        self._last_log_t = self._last_detail_t = -1e9
        self._burst_left = 0

    # ---- internals -----------------------------------------------------

    def _header(self) -> None:
        self.log.appendPlainText(
            f"{'t (s)':>8}  {'line on the wire':<22}"
            f"  dshot values       why"
        )

    def _show(self, frame) -> None:
        blocks = [
            describe(frame.values[i], frame.words[i], frame.demand[i], i)
            for i in (0, 1)
        ]
        self.detail.setText(
            "\n".join(blocks) + f"\n\nwire  {frame.line!r}   sent on {frame.reason}"
            + ("" if frame.sent else "   (port closed — nothing left the app)")
        )

    def _start_burst(self) -> None:
        self._burst_left = max(1, int(self.burst_frames))
        self.log.appendPlainText(f"--- burst of {self._burst_left} frames")

    def _copy(self) -> None:
        QGuiApplication.clipboard().setText(self.log.toPlainText())
