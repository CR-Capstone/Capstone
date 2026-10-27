"""Main window: a stack of control pages over one serial link and one engine.

Only the visible page drives the output. Switching pages disarms, and so does
losing the port, losing window focus, or pressing space. Arming is the single
gate between the UI and the motors.
"""
from __future__ import annotations

import argparse
import sys

from PySide6.QtCore import QEvent, Qt, QTimer
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QLabel, QLineEdit, QMainWindow, QMessageBox,
    QPlainTextEdit, QPushButton, QSizePolicy, QStatusBar, QTabWidget, QToolBar,
    QWidget,
)

from . import theme
from .engine import OutputEngine
from .link import SerialLink
from .pages import PAGES

STATUS_INTERVAL_MS = 100
BLUR_SETTLE_MS = 100


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Combat robot controls")
        self.resize(1500, 900)

        self.link = SerialLink()
        self.engine = OutputEngine(self.link)

        self.tabs = QTabWidget()
        self.pages = [page_type(self.engine, self.link) for page_type in PAGES]
        for page in self.pages:
            self.tabs.addTab(page, page.title)
            self.tabs.setTabToolTip(self.tabs.count() - 1, page.summary)
        self.tabs.currentChanged.connect(self._page_changed)
        self.setCentralWidget(self.tabs)

        self._build_toolbar()
        self.setStatusBar(QStatusBar())
        self.status_label = QLabel(objectName="mono")
        self.statusBar().addWidget(self.status_label, 1)

        self.engine.frame_sent.connect(self._on_frame)
        self.engine.armed_changed.connect(self._on_armed)
        self.link.opened.connect(lambda _d: self._update_arm_enabled())
        self.link.closed.connect(lambda _r: self._update_arm_enabled())
        self.link.failed.connect(self._on_link_failed)

        QApplication.instance().installEventFilter(self)

        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._refresh_status)
        self._status_timer.start(STATUS_INTERVAL_MS)

        self._page_changed(0)

    # ---- chrome --------------------------------------------------------

    def _build_toolbar(self) -> None:
        bar = QToolBar("Safety")
        bar.setMovable(False)
        self.addToolBar(bar)

        self.arm_button = QPushButton("ARM")
        self.arm_button.setObjectName("arm")
        self.arm_button.setCheckable(True)
        self.arm_button.setMinimumWidth(120)
        self.arm_button.toggled.connect(self._toggle_arm)
        bar.addWidget(self.arm_button)

        stop_button = QPushButton("STOP  (space)")
        stop_button.setObjectName("stop")
        stop_button.clicked.connect(lambda: self.panic("stop button"))
        bar.addWidget(stop_button)

        bar.addSeparator()
        self.confirm_arm = QCheckBox("Confirm before arming")
        self.confirm_arm.setChecked(True)
        bar.addWidget(self.confirm_arm)
        self.disarm_on_blur = QCheckBox("Disarm when the window loses focus")
        self.disarm_on_blur.setChecked(True)
        bar.addWidget(self.disarm_on_blur)

        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        bar.addWidget(spacer)
        self.banner = QLabel("DISARMED", objectName="banner")
        self._set_banner(False, "")
        bar.addWidget(self.banner)

    def _set_banner(self, armed: bool, detail: str) -> None:
        arming = armed and self.engine.state == "arming"
        color = theme.WARN if arming else theme.DANGER if armed else theme.OK
        if arming:
            text = f"ARMING · {self.engine.arming_status()}"
        else:
            text = f"ARMED · {detail}" if armed else "DISARMED"
        self.banner.setText(text)
        self.banner.setStyleSheet(
            f"background:{color}; color:#101923; padding:8px; border-radius:4px;"
        )

    # ---- wiring --------------------------------------------------------

    @property
    def page(self):
        return self.pages[self.tabs.currentIndex()]

    def _page_changed(self, index: int) -> None:
        for position, page in enumerate(self.pages):
            if position != index:
                page.on_deactivated()
        page = self.pages[index]
        if page.provides_output:
            self.engine.set_source(page.sample, page.title)
        else:
            self.engine.set_source(None, "")
        page.on_activated()
        self._update_arm_enabled()

    def _update_arm_enabled(self) -> None:
        ready = self.link.is_open and self.page.provides_output
        self.arm_button.setEnabled(ready)
        if not ready and self.arm_button.isChecked():
            self.arm_button.setChecked(False)

    def _toggle_arm(self, checked: bool) -> None:
        if not checked:
            self.engine.disarm("operator")
            return
        if self.confirm_arm.isChecked():
            answer = QMessageBox.question(
                self, "Arm the robot?",
                f"{self.page.title} is about to drive both motors.\n\n"
                "Everyone clear, robot restrained or on blocks?",
                QMessageBox.StandardButton.Cancel | QMessageBox.StandardButton.Yes,
                QMessageBox.StandardButton.Cancel,
            )
            if answer != QMessageBox.StandardButton.Yes:
                self.arm_button.setChecked(False)
                return
        if not self.engine.arm():
            self.arm_button.setChecked(False)

    def eventFilter(self, obj, event) -> bool:
        """Space and escape stop the robot from anywhere except a text field."""
        if (
            event.type() == QEvent.Type.KeyPress
            and self.isActiveWindow()
            and event.key() in (Qt.Key.Key_Space, Qt.Key.Key_Escape)
            and not isinstance(
                QApplication.focusWidget(), (QLineEdit, QPlainTextEdit)
            )
        ):
            self.panic("space" if event.key() == Qt.Key.Key_Space else "escape")
            return True
        return super().eventFilter(obj, event)

    def panic(self, reason: str) -> None:
        """Disarm now, from anywhere."""
        self.arm_button.setChecked(False)
        self.engine.disarm(reason)

    def _on_armed(self, armed: bool, reason: str) -> None:
        # Reflect the engine without re-entering _toggle_arm, which would ask
        # for confirmation again.
        self.arm_button.blockSignals(True)
        self.arm_button.setChecked(armed)
        self.arm_button.blockSignals(False)
        self.arm_button.setText("ARMED — click to stop" if armed else "ARM")
        self._set_banner(armed, reason)

    def _on_frame(self, frame) -> None:
        self.page.on_frame(frame)

    def _on_link_failed(self, message: str) -> None:
        self.statusBar().showMessage(message, 6000)
        self._update_arm_enabled()

    def _refresh_status(self) -> None:
        if self.engine.state == "arming":
            self._set_banner(True, "")
        port = self.link.port_name or "no port"
        lockout = self.engine.receiver_ready_in() if self.link.is_open else 0.0
        if lockout > 0.0:
            port += f" (receiver may be locked out {lockout:4.1f} s)"
        last = self.engine.last_frame
        wire = last.line.decode("ascii", "replace").strip() if last else "—"
        self.status_label.setText(
            f"{port}   eval {self.engine.measured_tick_rate():5.0f} Hz   "
            f"send {self.engine.measured_send_rate():5.1f} Hz   "
            f"sent {self.link.frames_sent}   held {self.engine.frames_held}   "
            f"dropped {self.link.frames_dropped}   "
            f"last {last.reason if last else '—'}  {wire!r}"
        )

    # ---- lifecycle -----------------------------------------------------

    def changeEvent(self, event) -> None:
        if (
            event.type() == event.Type.ActivationChange
            and not self.isActiveWindow()
            and self.disarm_on_blur.isChecked()
        ):
            # Focus moving to this app's own dialog (the arm confirmation) is
            # not leaving. Wayland reports no active window between the leave
            # and the enter, so look again once the move has settled.
            QTimer.singleShot(BLUR_SETTLE_MS, self._check_blur)
        super().changeEvent(event)

    def _check_blur(self) -> None:
        if self.disarm_on_blur.isChecked() and QApplication.activeWindow() is None:
            self.panic("window lost focus")

    def closeEvent(self, event) -> None:
        self.engine.disarm("closing")
        self.engine.stop()
        self.link.close("closing")
        super().closeEvent(event)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial device to open on start")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--smoke", action="store_true",
                        help="build the window, run briefly, exit")
    args = parser.parse_args(argv)

    app = QApplication(sys.argv[:1])
    app.setStyleSheet(theme.STYLE)
    window = MainWindow()
    if args.port:
        window.link.open(args.port, args.baud)
    window.show()
    if args.smoke:
        QTimer.singleShot(600, app.quit)
        app.exec()
        window.close()
        return 0
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
