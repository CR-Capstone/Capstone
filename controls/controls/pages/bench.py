"""Stage 1 — bench. Direct per-motor throttle with the robot on blocks.

Two ways to drive: the sliders, for poking at one value, and the canned tests
in :mod:`controls.sequences`, for the checks every new ESC pairing needs. A
running test owns the output and returns to zero when it ends; the sliders
take over again as soon as it stops.

The DShot command buttons are the ESC configuration path and only work while
disarmed, because an ESC ignores commands with the motor spinning.
"""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QListWidget, QListWidgetItem, QPushButton, QSlider, QVBoxLayout,
)

from .. import dshot, theme
from ..scope import MotorReadout, Scope
from ..sequences import TESTS, custom_ramp
from .base import ControlPage, reverse_warning

COMMANDS = (
    ("Beep 1", dshot.Command.BEEP1),
    ("Beep 3", dshot.Command.BEEP3),
    ("Spin dir 1", dshot.Command.SPIN_DIRECTION_1),
    ("Spin dir 2", dshot.Command.SPIN_DIRECTION_2),
    ("3-D on", dshot.Command.MODE_3D_ON),
    ("3-D off", dshot.Command.MODE_3D_OFF),
    ("Save settings", dshot.Command.SAVE_SETTINGS),
    ("Motor stop", dshot.Command.MOTOR_STOP),
)
CUSTOM_LABEL = "Custom ramp (uses the fields below)"


class ThrottleSlider(QGroupBox):
    """One vertical -100..100 % slider with a live percentage label."""

    def __init__(self, index: int) -> None:
        super().__init__(f"Motor {index + 1}")
        layout = QVBoxLayout(self)
        self.setFixedWidth(88)
        self.slider = QSlider(Qt.Orientation.Vertical)
        self.slider.setRange(-100, 100)
        self.slider.setValue(0)
        self.slider.setTickInterval(25)
        self.slider.setTickPosition(QSlider.TickPosition.TicksBothSides)
        self.label = QLabel("0 %", objectName="mono")
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.slider, 1, Qt.AlignmentFlag.AlignHCenter)
        layout.addWidget(self.label)
        self.slider.valueChanged.connect(lambda v: self.label.setText(f"{v:+d} %"))

    def demand(self) -> float:
        return self.slider.value() / 100.0


class BenchPage(ControlPage):
    title = "1 · Bench"
    stage = 1
    summary = "Direct throttle, canned bring-up tests, and ESC configuration."

    def __init__(self, engine, link) -> None:
        super().__init__(engine, link)
        self._running = None
        self._started_at: float | None = None

        layout = QHBoxLayout(self)
        sliders = QHBoxLayout()
        self.sliders = [ThrottleSlider(0), ThrottleSlider(1)]
        for widget in self.sliders:
            sliders.addWidget(widget)
        self.sliders[0].slider.valueChanged.connect(self._mirror)
        layout.addLayout(sliders, 0)

        middle = QVBoxLayout()
        layout.addLayout(middle, 2)
        middle.addWidget(self._tests_box())
        middle.addWidget(self._options_box())
        middle.addStretch(1)

        right = QVBoxLayout()
        layout.addLayout(right, 2)
        right.addWidget(self._command_box())
        self.readouts = [MotorReadout(0), MotorReadout(1)]
        for widget in self.readouts:
            right.addWidget(widget)
        self.scope = Scope(window=6.0)
        right.addWidget(self.scope, 1)
        self.warning = QLabel(objectName="mono")
        self.warning.setWordWrap(True)
        self.warning.setStyleSheet(f"color:{theme.WARN}")
        right.addWidget(self.warning)
        self._select_test()

    # ---- widgets -------------------------------------------------------

    def _tests_box(self) -> QGroupBox:
        box = QGroupBox("Bring-up tests")
        layout = QVBoxLayout(box)

        self.tests = QListWidget()
        for sequence in TESTS:
            item = QListWidgetItem(sequence.name)
            item.setData(Qt.ItemDataRole.UserRole, sequence)
            item.setToolTip(sequence.watch_for)
            self.tests.addItem(item)
        custom = QListWidgetItem(CUSTOM_LABEL)
        custom.setData(Qt.ItemDataRole.UserRole, None)
        self.tests.addItem(custom)
        self.tests.setMaximumHeight(200)
        self.tests.setCurrentRow(0)
        self.tests.currentRowChanged.connect(self._select_test)
        self.tests.itemDoubleClicked.connect(lambda _i: self.run_button.setChecked(True))
        layout.addWidget(self.tests)

        self.watch_for = QLabel(objectName="mono")
        self.watch_for.setWordWrap(True)
        self.watch_for.setMinimumHeight(46)
        layout.addWidget(self.watch_for)

        row = QHBoxLayout()
        self.run_button = QPushButton("Run test")
        self.run_button.setCheckable(True)
        self.run_button.toggled.connect(self._toggle_run)
        self.progress = QLabel("idle", objectName="mono")
        row.addWidget(self.run_button)
        row.addWidget(self.progress, 1)
        layout.addLayout(row)
        return box

    def _options_box(self) -> QGroupBox:
        box = QGroupBox("Sliders and custom ramp")
        form = QFormLayout(box)
        self.linked = QCheckBox("Motor 2 follows motor 1")
        self.mirror_sign = QCheckBox("Mirrored (motor 2 takes the opposite sign)")
        self.mirror_sign.setChecked(True)
        self.mirror_sign.toggled.connect(self._select_test)
        self.ramp_target = QDoubleSpinBox()
        self.ramp_target.setRange(-100.0, 100.0)
        self.ramp_target.setValue(25.0)
        self.ramp_target.setSuffix(" %")
        self.ramp_target.valueChanged.connect(self._select_test)
        self.ramp_seconds = QDoubleSpinBox()
        self.ramp_seconds.setRange(0.2, 30.0)
        self.ramp_seconds.setValue(4.0)
        self.ramp_seconds.setSuffix(" s")
        self.ramp_seconds.valueChanged.connect(self._select_test)
        zero = QPushButton("Zero both")
        zero.clicked.connect(self.zero)
        buttons = QHBoxLayout()
        buttons.addWidget(zero)
        buttons.addStretch(1)

        form.addRow(self.linked)
        form.addRow(self.mirror_sign)
        form.addRow("Ramp target", self.ramp_target)
        form.addRow("Ramp time", self.ramp_seconds)
        form.addRow(buttons)
        return box

    def _command_box(self) -> QGroupBox:
        box = QGroupBox("DShot commands (disarmed only)")
        grid = QGridLayout(box)
        for position, (label, value) in enumerate(COMMANDS):
            button = QPushButton(label)
            button.clicked.connect(lambda _=False, v=value: self.engine.queue_command(v))
            grid.addWidget(button, position // 2, position % 2)
        note = QLabel(
            "Sent ten times to both motors. ESCs ignore these while spinning.",
            objectName="mono",
        )
        note.setWordWrap(True)
        grid.addWidget(note, len(COMMANDS) // 2, 0, 1, 2)
        return box

    # ---- tests ---------------------------------------------------------

    def selected_sequence(self):
        """The chosen test, building the custom ramp from the fields if needed."""
        item = self.tests.currentItem()
        if item is None:
            return None
        sequence = item.data(Qt.ItemDataRole.UserRole)
        if sequence is not None:
            return sequence
        return custom_ramp(
            self.ramp_target.value() / 100.0,
            self.ramp_seconds.value(),
            self.mirror_sign.isChecked(),
        )

    def _select_test(self, *_args) -> None:
        sequence = self.selected_sequence()
        if sequence is None:
            return
        if self.run_button.isChecked():
            self.run_button.setChecked(False)
        count = len(sequence.steps)
        self.watch_for.setText(
            f"Watch for: {sequence.watch_for}\n"
            f"{count} step{'' if count == 1 else 's'}, "
            f"{sequence.duration():.1f} s total."
        )
        lows = [0.0, 0.0]
        for index in range(81):
            demand = sequence.sample(sequence.duration() * index / 80.0)
            lows = [min(lows[i], demand[i]) for i in (0, 1)]
        self.warning.setText(
            reverse_warning(self.engine, (lows[0] < 0.0, lows[1] < 0.0))
        )

    def _toggle_run(self, checked: bool) -> None:
        if not checked:
            self._running = None
            self._started_at = None
            self.run_button.setText("Run test")
            self.progress.setText("idle")
            return
        self._running = self.selected_sequence()
        self._started_at = None
        self.run_button.setText("Stop test")
        if not self.engine.armed:
            self.progress.setText("waiting — arm to start")

    # ---- behaviour -----------------------------------------------------

    def _mirror(self, value: int) -> None:
        if not self.linked.isChecked():
            return
        self.sliders[1].slider.setValue(-value if self.mirror_sign.isChecked() else value)

    def zero(self) -> None:
        self.run_button.setChecked(False)
        for widget in self.sliders:
            widget.slider.setValue(0)

    def sample(self, t: float) -> tuple[float, float]:
        if self._running is not None:
            if self._started_at is None:
                self._started_at = t
            return self._running.sample(t - self._started_at)
        return self.sliders[0].demand(), self.sliders[1].demand()

    def on_frame(self, frame) -> None:
        for index, readout in enumerate(self.readouts):
            readout.set_state(frame.demand[index], frame.values[index])
        self.scope.add(frame)
        if self._running is not None and self._started_at is not None:
            elapsed = frame.t - self._started_at
            step = self._running.step_at(elapsed)
            if step is None:
                self.progress.setText("finished")
                self.run_button.setChecked(False)
            else:
                self.progress.setText(
                    f"{elapsed:5.1f} / {self._running.duration():.1f} s   {step.describe()}"
                )

    def on_activated(self) -> None:
        self.zero()
        self.scope.clear()
        self._select_test()

    def on_deactivated(self) -> None:
        self.zero()
