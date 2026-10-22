"""Stage 0 — link. Open the port, agree on the wire format, watch raw traffic.

Nothing here commands a motor. The page exists so the protocol can be proven
against the firmware before anything spins: every setting that changes the
bytes on the wire lives on this page, and the console shows exactly what went
out and came back.
"""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QPlainTextEdit, QPushButton, QSpinBox,
    QVBoxLayout,
)

from .. import dshot, protocol, theme
from ..engine import (
    HOLD_TIMEOUT_RANGE, KEEPALIVE_RANGE, RATE_RANGE, keepalive_margin,
)
from ..link import BAUD_RATES, DEFAULT_BAUD, available_ports
from .base import ControlPage

CONSOLE_LINES = 400


class MotorMapBox(QGroupBox):
    """Editor for one :class:`~controls.dshot.MotorMap`."""

    def __init__(self, index: int, mapping: dshot.MotorMap, on_change) -> None:
        super().__init__(f"Motor {index + 1} mapping")
        self.mapping = mapping
        self._notify = on_change
        form = QFormLayout(self)

        self.invert = QCheckBox("Invert sign")
        self.invert.setChecked(mapping.invert)
        self.three_d = QCheckBox("3-D range (48–1047 rev · 1048 neutral · 1049–2047 fwd)")
        self.three_d.setChecked(mapping.three_d)
        self.deadband = QDoubleSpinBox()
        self.deadband.setRange(0.0, 0.5)
        self.deadband.setSingleStep(0.01)
        self.deadband.setDecimals(3)
        self.deadband.setValue(mapping.deadband)
        self.limit = QDoubleSpinBox()
        self.limit.setRange(0.0, 1.0)
        self.limit.setSingleStep(0.05)
        self.limit.setDecimals(2)
        self.limit.setValue(mapping.limit)
        self.limit.setToolTip("Hard ceiling on this motor, applied after the page's demand.")

        form.addRow(self.invert)
        form.addRow(self.three_d)
        form.addRow("Deadband", self.deadband)
        form.addRow("Output limit", self.limit)

        for widget in (self.invert, self.three_d):
            widget.toggled.connect(self._apply)
        for widget in (self.deadband, self.limit):
            widget.valueChanged.connect(self._apply)

    def _apply(self, *_args) -> None:
        self.mapping.invert = self.invert.isChecked()
        self.mapping.three_d = self.three_d.isChecked()
        self.mapping.deadband = self.deadband.value()
        self.mapping.limit = self.limit.value()
        self._notify()


class LinkPage(ControlPage):
    title = "0 · Link"
    stage = 0
    summary = "Port, wire format and raw traffic. No motor output from this page."
    provides_output = False

    def __init__(self, engine, link) -> None:
        super().__init__(engine, link)
        layout = QHBoxLayout(self)
        left = QVBoxLayout()
        layout.addLayout(left, 3)

        left.addWidget(self._port_box())
        left.addWidget(self._timing_box())
        left.addWidget(self._arming_box())
        maps = QHBoxLayout()
        for index, mapping in enumerate(engine.config.maps):
            maps.addWidget(MotorMapBox(index, mapping, self._preview))
        left.addLayout(maps)
        left.addWidget(self._preview_box())
        left.addStretch(1)

        right = QVBoxLayout()
        layout.addLayout(right, 4)
        right.addWidget(QLabel("Serial console", objectName="section"))
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(CONSOLE_LINES)
        right.addWidget(self.console, 1)

        send_row = QHBoxLayout()
        self.manual = QLineEdit()
        self.manual.setPlaceholderText("raw line to send, e.g.  1048 1048")
        self.manual.returnPressed.connect(self._send_manual)
        send = QPushButton("Send line")
        send.clicked.connect(self._send_manual)
        clear = QPushButton("Clear")
        clear.clicked.connect(self.console.clear)
        send_row.addWidget(self.manual, 1)
        send_row.addWidget(send)
        send_row.addWidget(clear)
        right.addLayout(send_row)

        link.received.connect(self._on_received)
        link.opened.connect(lambda d: self._log(f"-- opened {d}"))
        link.closed.connect(lambda r: self._log(f"-- closed ({r})"))
        link.failed.connect(lambda r: self._log(f"!! {r}"))
        self.refresh_ports()
        self._preview()

    # ---- widgets -------------------------------------------------------

    def _port_box(self) -> QGroupBox:
        box = QGroupBox("Port")
        grid = QGridLayout(box)
        self.ports = QComboBox()
        self.ports.setMinimumWidth(260)
        self.baud = QComboBox()
        for rate in BAUD_RATES:
            self.baud.addItem(str(rate), rate)
        self.baud.setCurrentText(str(DEFAULT_BAUD))
        self.baud.setToolTip("Ignored by the XIAO's native USB serial.")
        rescan = QPushButton("Rescan")
        rescan.clicked.connect(self.refresh_ports)
        self.connect_button = QPushButton("Connect")
        self.connect_button.setCheckable(True)
        self.connect_button.toggled.connect(self._toggle_connection)
        grid.addWidget(QLabel("Device"), 0, 0)
        grid.addWidget(self.ports, 0, 1)
        grid.addWidget(rescan, 0, 2)
        grid.addWidget(QLabel("Baud"), 1, 0)
        grid.addWidget(self.baud, 1, 1)
        grid.addWidget(self.connect_button, 1, 2)
        return box

    def _timing_box(self) -> QGroupBox:
        box = QGroupBox("Timing")
        form = QFormLayout(box)
        config = self.engine.config

        self.rate = QSpinBox()
        self.rate.setRange(*RATE_RANGE)
        self.rate.setValue(config.rate_hz)
        self.rate.setSuffix(" Hz")
        self.rate.setToolTip(
            "How often the active page is evaluated. This is a ceiling on the "
            "send rate, not the send rate: a frame goes out only when the "
            "values changed."
        )
        self.rate.valueChanged.connect(self._apply_rate)

        self.keepalive = QDoubleSpinBox()
        self.keepalive.setRange(*KEEPALIVE_RANGE)
        self.keepalive.setDecimals(2)
        self.keepalive.setSingleStep(0.05)
        self.keepalive.setValue(config.keepalive_s)
        self.keepalive.setSuffix(" s")
        self.keepalive.setToolTip(
            "Resend the held frame at least this often. Keep it well under the "
            "firmware's hold timeout."
        )

        self.hold_timeout = QDoubleSpinBox()
        self.hold_timeout.setRange(*HOLD_TIMEOUT_RANGE)
        self.hold_timeout.setDecimals(2)
        self.hold_timeout.setSingleStep(0.1)
        self.hold_timeout.setValue(config.hold_timeout_s)
        self.hold_timeout.setSuffix(" s")
        self.hold_timeout.setToolTip(
            "How long the receiver forwards the last value before cutting the "
            "ESCs (FAILSAFE_TIMEOUT_MS in receiver.c). Match the firmware."
        )

        self.change_counts = QSpinBox()
        self.change_counts.setRange(1, 200)
        self.change_counts.setValue(config.change_counts)
        self.change_counts.setSuffix(" counts")
        self.change_counts.setToolTip(
            "Smallest DShot change worth a packet. Raising it quantises a "
            "waveform and cuts the traffic it costs. A motor starting or "
            "stopping always goes out."
        )

        self.timing_note = QLabel(objectName="mono")
        self.timing_note.setWordWrap(True)
        self.timing_note.setMinimumHeight(78)   # room for the margin warning
        self.timing_note.setAlignment(
            Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft
        )

        form.addRow("Evaluate at", self.rate)
        form.addRow("Keepalive", self.keepalive)
        form.addRow("Firmware hold", self.hold_timeout)
        form.addRow("Change threshold", self.change_counts)
        form.addRow(self.timing_note)

        for widget in (self.keepalive, self.hold_timeout, self.change_counts):
            widget.valueChanged.connect(self._apply_format)
        return box

    def _arming_box(self) -> QGroupBox:
        box = QGroupBox("Receiver lockout and arming sequence")
        form = QFormLayout(box)
        config = self.engine.config

        def seconds(value: float, low: float, high: float, tip: str) -> QDoubleSpinBox:
            spin = QDoubleSpinBox()
            spin.setRange(low, high)
            spin.setDecimals(2)
            spin.setSingleStep(0.1)
            spin.setValue(value)
            spin.setSuffix(" s")
            spin.setToolTip(tip)
            spin.valueChanged.connect(self._apply_format)
            return spin

        self.lockout = seconds(
            config.lockout_s, 0.0, 60.0,
            "After a failsafe the receiver ignores everything for this long "
            "(LOCKOUT_MS in receiver.c). Arming waits it out.",
        )
        self.arm_zero = seconds(
            config.arm_zero_s, 0.0, 10.0,
            "Value 0 on both motors so the ESCs arm.",
        )
        self.arm_neutral = seconds(
            config.arm_neutral_s, 0.0, 10.0,
            "Then idle (3-D neutral 1048) before the page takes over.",
        )
        form.addRow("Receiver lockout", self.lockout)
        form.addRow("Arm: hold 0 for", self.arm_zero)
        form.addRow("Then neutral for", self.arm_neutral)
        return box

    def _preview_box(self) -> QGroupBox:
        box = QGroupBox("Contract preview")
        layout = QVBoxLayout(box)
        self.preview = QLabel(objectName="mono")
        self.preview.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.preview.setWordWrap(True)
        layout.addWidget(self.preview)
        return box

    # ---- behaviour -----------------------------------------------------

    def refresh_ports(self) -> None:
        """Re-enumerate serial devices, keeping the current selection if present."""
        current = self.ports.currentData()
        self.ports.clear()
        for device, description in available_ports():
            self.ports.addItem(f"{device} — {description}", device)
        if current is not None:
            index = self.ports.findData(current)
            if index >= 0:
                self.ports.setCurrentIndex(index)
        if self.ports.count() == 0:
            self.ports.addItem("no serial ports found", None)

    def _toggle_connection(self, checked: bool) -> None:
        if not checked:
            self.link.close("operator")
            self.connect_button.setText("Connect")
            return
        device = self.ports.currentData()
        if device is None:
            self._log("!! no port selected")
            self.connect_button.setChecked(False)
            return
        if self.link.open(device, self.baud.currentData()):
            self.connect_button.setText("Disconnect")
        else:
            self.connect_button.setChecked(False)

    def _apply_rate(self, rate: int) -> None:
        self.engine.apply_rate(rate)
        self._preview()

    def _apply_format(self, *_args) -> None:
        config = self.engine.config
        config.lockout_s = self.lockout.value()
        config.arm_zero_s = self.arm_zero.value()
        config.arm_neutral_s = self.arm_neutral.value()
        config.keepalive_s = self.keepalive.value()
        config.hold_timeout_s = self.hold_timeout.value()
        config.change_counts = self.change_counts.value()
        self._preview()

    def _preview(self) -> None:
        """Show the literal bytes a half-throttle frame produces right now."""
        config = self.engine.config
        demands = (0.5, -0.5)
        values = [m.value(d) for m, d in zip(config.maps, demands)]
        idle = [m.idle for m in config.maps]
        words = [dshot.frame(v) for v in values]
        line = protocol.encode(values[0], values[1])
        self.preview.setText(
            f"demand   {demands[0]:+.2f}  {demands[1]:+.2f}\n"
            f"on wire  {line!r}      armed idle {idle[0]} {idle[1]}\n"
            f"ESC gets 0x{words[0]:04X}  0x{words[1]:04X}   (frame built by the receiver)\n"
            f"arming   0 for {config.arm_zero_s:.1f} s, then {idle[0]} {idle[1]} "
            f"for {config.arm_neutral_s:.1f} s"
        )
        idle = len(line) / config.keepalive_s
        busy = len(line) * config.rate_hz
        margin = keepalive_margin(config)
        self.timing_note.setText(
            f"Holding still: one {len(line)}-byte line every "
            f"{config.keepalive_s:.2f} s, about {idle:.0f} B/s.\n"
            f"Worst case, every evaluation changes something: "
            f"{config.rate_hz} lines/s, about {busy:,.0f} B/s. A continuous "
            f"waveform sits at the worst case unless the change threshold "
            f"is raised."
            + (f"\n{margin}" if margin else "")
        )
        self.timing_note.setStyleSheet(f"color:{theme.WARN}" if margin else "")

    def _send_manual(self) -> None:
        text = self.manual.text().strip()
        if not text:
            return
        if not self.link.is_open:
            self._log("!! port is closed")
            return
        self.link.write((text + protocol.TERMINATOR).encode("ascii", "replace"))
        self._log(f">> {text}")
        self.manual.clear()

    def _on_received(self, text: str) -> None:
        self._log(f"<< {text}")

    def _log(self, text: str) -> None:
        self.console.appendPlainText(text)
