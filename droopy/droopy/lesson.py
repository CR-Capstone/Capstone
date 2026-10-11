"""A visual, plain-language bridge from gait controls to robot motion.

``GuidedLesson`` deliberately does not control the simulator.  It previews the
operator's open-loop command and explains immutable completed ticks supplied by
the parent UI.
"""
from __future__ import annotations

from dataclasses import asdict
import copy
import math

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import (
    QButtonGroup, QCheckBox, QComboBox, QHBoxLayout, QLabel, QPushButton, QScrollArea,
    QVBoxLayout, QWidget,
)

from .control import Command
from .force_story import TorqueMotionView
from .contact_view import ContactSchematic


STAGES = (
    ("Choose gait", "Pick the open-loop rotor pattern you want to try."),
    ("Phase → targets", "A phase cursor turns the chosen waveform into two signed speed targets."),
    ("Error → torque", "Each motor pushes its measured rotor speed toward its target."),
    ("Reaction + gyro → rock", "Motor reaction and gyroscopic effects rotate the chassis; they do not directly push it forward."),
    ("Contact → push", "Only a loaded foot can exchange force with the floor and push the chassis."),
    ("Accumulated motion", "Many tiny contact impulses can build velocity and displacement."),
)


def _copy_command(command: Command | None) -> Command | None:
    return None if command is None else Command(**asdict(command))


def _pair(values) -> str:
    return f"({float(values[0]):+.3f}, {float(values[1]):+.3f})"


class PhaseWaveform(QWidget):
    """Motor-target magnitude preview; direction is shown separately."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.command = Command()
        self.time_s = 0.0
        self.max_speed = 450.
        self.setMinimumHeight(190)
        self.setToolTip('Magnitude height is absolute speed. Direction strips show +, − or stopped over the full cycle; each sign is about its rotor axis.')

    def set_state(self, command: Command, time_s: float, max_speed: float = 450.) -> None:
        self.command = _copy_command(command) or Command()
        self.time_s = float(time_s)
        self.max_speed = float(max_speed)
        signed=self.command.targets(self.time_s,self.max_speed)
        self.direction_text='At cursor: L '+self.direction_label(signed[0])+' / R '+self.direction_label(signed[1])
        self.setAccessibleDescription(self.direction_text+'; full-cycle direction strips mark reversals independently of speed magnitude.')
        self.update()

    @staticmethod
    def direction_label(value):
        return '+ axis' if value>1e-9 else ('− axis' if value < -1e-9 else 'stopped')

    def signed_cycle_values(self, count=81):
        frequency=max(self.command.frequency,1e-12)
        return [self.command.targets(i/(count-1)/frequency,self.max_speed) for i in range(count)]

    def cycle_directions(self, count=81):
        return [tuple(self.direction_label(v) for v in pair) for pair in self.signed_cycle_values(count)]

    def cycle_values(self, count=81):
        return [tuple(abs(v) for v in pair) for pair in self.signed_cycle_values(count)]

    def paintEvent(self, event):  # noqa: N802 - Qt API
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#0e1722"))
        box = QRectF(35, 30, max(20, self.width()-48), max(30, self.height()-106))
        p.setPen(QPen(QColor("#405267"), 1))
        p.drawLine(box.left(), box.bottom(), box.right(), box.bottom())
        dash = QPen(QColor("#334559"), 1, Qt.PenStyle.DashLine)
        p.setPen(dash)
        for q in range(5):
            x = box.left()+q*box.width()/4
            p.drawLine(x, box.top(), x, box.bottom())
        c = self.command
        values = self.cycle_values()
        scale = max(1.0, max(abs(v) for pair in values for v in pair))
        for side, color in ((0, "#ff796d"), (1, "#74b9ff")):
            points = [QPointF(box.left()+i*box.width()/80,
                              box.bottom()-pair[side]/scale*box.height()*.95)
                      for i, pair in enumerate(values)]
            p.setPen(QPen(QColor(color), 2))
            p.drawPolyline(QPolygonF(points))
        frequency = c.frequency if c.frequency > 0 else 1.0
        phase_fraction = (self.time_s*frequency) % 1.0
        x = box.left()+phase_fraction*box.width()
        p.setPen(QPen(QColor("#ffd166"), 2))
        p.drawLine(x, box.top(), x, box.bottom())
        p.setPen(QColor("#aebed0"))
        p.drawText(8,18,'TARGET MAGNITUDE |ω*| · L coral / R blue')
        p.drawText(8,int(box.bottom()),'0')
        p.drawText(8,self.height()-63,getattr(self,'direction_text','At cursor: L stopped / R stopped'))
        directions=self.cycle_directions()
        for side,label in enumerate(('L dir','R dir')):
            yy=self.height()-52+side*16
            p.setPen(QColor('#aebed0'));p.drawText(1,yy+10,label)
            for i,pair in enumerate(directions[:-1]):
                value=pair[side]
                color='#eab65e' if value=='+ axis' else '#708dc3' if value=='− axis' else '#47515c'
                p.fillRect(QRectF(box.left()+i*box.width()/80,yy,box.width()/80+1,12),QColor(color))
                if i==0 or pair[side]!=directions[i-1][side]:
                    p.setPen(QColor('#101923'));p.drawText(int(box.left()+i*box.width()/80),yy+10,'+' if value=='+ axis' else '−' if value=='− axis' else '0')
        p.setPen(QColor('#aebed0'))
        for i,text in enumerate(('0','¼','½','¾','1 cycle')):
            p.drawText(QRectF(box.left()+i*box.width()/4-18,self.height()-20,40,18),Qt.AlignmentFlag.AlignCenter,text)
        p.end()


class RotorBars(QWidget):
    """Absolute speed bars with rotation direction labelled independently."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.values = (0.0, 0.0)
        self.actions = ("holding", "holding")
        self.limit = 450.0
        self.setMinimumHeight(148)
        self.setAccessibleName("Left and right speed magnitude with separate rotation direction")

    def set_state(self, values, actions=("holding", "holding"), limit=450.0):
        self.values = tuple(float(v) for v in values)
        self.actions = tuple(actions)
        self.limit = max(1.0, float(limit))
        self.setAccessibleDescription(
            f"left {self.values[0]:+.3f}, {self.actions[0]}; right {self.values[1]:+.3f}, {self.actions[1]}")
        self.update()

    def paintEvent(self, event):  # noqa: N802
        p=QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(),QColor("#0e1722"))
        p.setPen(QColor("#aebed0")); p.drawText(10,18,"SPEED MAGNITUDE  ·  longer bar = faster")
        for row,(name,value,action,color) in enumerate(zip(("LEFT","RIGHT"),self.values,self.actions,("#ff796d","#74b9ff"))):
            y=45+row*57; start=80.; available=max(30.,self.width()-100.)
            p.setPen(QColor("#405267")); p.drawLine(start,y,start+available,y)
            end=start+min(1.,abs(value)/self.limit)*available
            p.setPen(QPen(QColor(color),9)); p.drawLine(start,y,end,y)
            p.setPen(QColor("#d6e0ec")); p.drawText(10,y+5,name)
            p.drawText(12,y+24,f"{abs(value):.1f} rad/s · {'+ axis' if value>1e-9 else '− axis' if value< -1e-9 else 'stopped'} · {action}")
        p.end()


class GuidedLesson(QWidget):
    """Six-stage lesson panel driven entirely by parent-supplied simulation state."""

    experiment_requested = Signal(str)
    step_requested = Signal(float)
    inspect_requested = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(390, 500)
        self._sim = None
        self._latest_diagnostic = None
        self._latest_consumed_command = None
        self._diagnostic = None  # explicitly captured evidence
        self._captured_config = None
        self._captured_state = None
        self._captured_impulses = None
        self._consumed_command = None
        self._pending_command = Command()
        self.body_forward_impulse_ns = 0.0
        self.world_impulse_ns = np.zeros(3, dtype=float)
        self.recorded_ticks = 0
        self._stage = 0

        outer = QVBoxLayout(self); outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea(); scroll.setWidgetResizable(True); outer.addWidget(scroll)
        page = QWidget(); page.setMinimumWidth(360); scroll.setWidget(page)
        layout = QVBoxLayout(page)
        title = QLabel("FROM SETPOINT TO SHUFFLE"); title.setObjectName("section"); layout.addWidget(title)
        note = QLabel("Follow one idea at a time. This is an illustrative waveform picked by the operator—not an inverse solution for forward motion.")
        note.setWordWrap(True); layout.addWidget(note)

        presets = QHBoxLayout()
        for text, key in (("Try demo", "demo"), ("Constant", "constant"), ("No grip", "no_grip")):
            button = QPushButton(text); button.setObjectName(f"lesson_{key}")
            button.clicked.connect(lambda checked=False, k=key: self.experiment_requested.emit(k))
            presets.addWidget(button)
        layout.addLayout(presets)

        self.stage_select=QComboBox(); self.stage_select.addItems([f'{i+1}. {s[0]}' for i,s in enumerate(STAGES)])
        self.stage_select.currentIndexChanged.connect(self.select_stage); layout.addWidget(self.stage_select)
        navigation=QHBoxLayout()
        previous=QPushButton('← Previous'); previous.clicked.connect(lambda:self.stage_select.setCurrentIndex(max(0,self._stage-1)))
        following=QPushButton('Next →'); following.clicked.connect(lambda:self.stage_select.setCurrentIndex(min(5,self._stage+1)))
        navigation.addWidget(previous); navigation.addWidget(following); layout.addLayout(navigation)
        self.stage_group = QButtonGroup(self); self.stage_group.setExclusive(True)
        self.stage_buttons = []
        for index, (name, _intro) in enumerate(STAGES):
            button = QPushButton(f"{index+1}. {name}"); button.setCheckable(True)
            button.setObjectName(f"lesson_stage_{index+1}")
            button.clicked.connect(lambda checked=False, i=index: self.select_stage(i))
            self.stage_group.addButton(button, index); self.stage_buttons.append(button); button.hide()
        self.stage_buttons[0].setChecked(True)

        cycle = QLabel("ONE FULL CYCLE:  1 START / CROSSING  →  2 LEFT PEAK  →  3 CROSSING  →  4 RIGHT PEAK  →  START")
        cycle.setWordWrap(True); layout.addWidget(cycle)
        samples = QLabel("These are four sample moments in the operator's command cycle—not guaranteed physical footsteps.")
        samples.setWordWrap(True); layout.addWidget(samples)
        self.rotor_bars = RotorBars(); layout.addWidget(self.rotor_bars)
        self.math_toggle = QCheckBox("Show waveform + torque + math")
        self.math_toggle.setChecked(False); layout.addWidget(self.math_toggle)
        self.waveform = PhaseWaveform(); self.waveform.setVisible(False); layout.addWidget(self.waveform)
        self.torque_view=TorqueMotionView();self.torque_view.hide();layout.addWidget(self.torque_view)
        self.torque_view.inspect_requested.connect(self.inspect_requested)
        self.math_toggle.toggled.connect(self.torque_view.setVisible)
        self.math_toggle.toggled.connect(self.waveform.setVisible)
        self.math_toggle.toggled.connect(lambda _checked: self._refresh_pending())
        self.contact_view = ContactSchematic(); layout.addWidget(self.contact_view)
        self.contact_view.inspect_requested.connect(self.inspect_requested)
        self.contact_legend=QLabel('Yellow dots = loaded feet carrying vertical support. Gray dots = unloaded. Dashed ghosts below show each foot’s horizontal floor push; cyan is the total.')
        self.contact_legend.setWordWrap(True);layout.insertWidget(layout.indexOf(self.rotor_bars),self.contact_legend)
        capture_row = QHBoxLayout()
        self.capture_button = QPushButton("Capture current step")
        self.capture_button.setObjectName("lesson_capture")
        self.capture_button.clicked.connect(self.capture_current); capture_row.addWidget(self.capture_button)
        self.live_check = QCheckBox("Live capture")
        self.live_check.setToolTip("Off by default so equations do not jump while you read them.")
        self.live_check.toggled.connect(lambda checked: self.capture_current() if checked else None)
        capture_row.addWidget(self.live_check); layout.insertLayout(layout.indexOf(self.waveform),capture_row)
        self.provenance_label = QLabel(); self.provenance_label.setWordWrap(True)
        self.provenance_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.provenance_label)
        self.detail = QLabel(); self.detail.setObjectName("lesson_detail"); self.detail.setWordWrap(True)
        self.detail.setTextFormat(Qt.TextFormat.PlainText); self.detail.setMinimumHeight(92)
        self.detail.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.detail.setAlignment(Qt.AlignmentFlag.AlignTop); layout.addWidget(self.detail)
        equation_heading = QLabel("ONE EQUATION / FIXED CAPTURE"); equation_heading.setObjectName("section")
        layout.addWidget(equation_heading)
        self.equation_label = QLabel(); self.equation_label.setObjectName("lesson_equation")
        self.equation_label.setTextFormat(Qt.TextFormat.PlainText)
        self.equation_label.setWordWrap(False); self.equation_label.setMinimumHeight(32)
        self.equation_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.equation_label.setStyleSheet("font-family: monospace; background:#182536; padding:8px;")
        equation_scroll=QScrollArea(); equation_scroll.setWidgetResizable(True); equation_scroll.setFixedHeight(92)
        equation_scroll.setWidget(self.equation_label); layout.addWidget(equation_scroll)
        equation_scroll.hide(); self.math_toggle.toggled.connect(equation_scroll.setVisible)
        equation_heading.setVisible(False); self.equation_label.setVisible(False)
        self.math_toggle.toggled.connect(equation_heading.setVisible)
        self.math_toggle.toggled.connect(self.equation_label.setVisible)
        self.pending_label = QLabel(); self.pending_label.setWordWrap(False)
        self.pending_label.setStyleSheet("font-family: monospace; color:#9fb2c7;")
        layout.addWidget(self.pending_label)
        self.inspect_button = QPushButton("Open detailed physics inspector")
        self.inspect_button.clicked.connect(self._inspect); layout.addWidget(self.inspect_button)
        self.step_button = QPushButton("Advance ¼ cycle")
        self.step_button.setObjectName("lesson_quarter_step"); self.step_button.clicked.connect(self._request_quarter_cycle)
        layout.addWidget(self.step_button)
        # Keep the lesson's explanation and deliberate stepping visible above
        # supporting pictures; the topic selector replaces six bulky buttons.
        for widget in (self.detail,self.provenance_label,self.step_button):
            layout.removeWidget(widget)
        layout.insertWidget(5,self.detail)
        layout.insertWidget(6,self.step_button)
        layout.insertWidget(7,self.provenance_label)
        layout.addStretch()
        self._refresh()

    @property
    def accumulated_body_forward_impulse_ns(self) -> float:
        return self.body_forward_impulse_ns

    @property
    def accumulated_world_impulse_ns(self) -> tuple[float, float, float]:
        return tuple(float(x) for x in self.world_impulse_ns)

    def select_stage(self, index: int) -> None:
        self._stage = max(0, min(len(STAGES)-1, int(index)))
        self.stage_buttons[self._stage].setChecked(True)
        if self.stage_select.currentIndex()!=self._stage: self.stage_select.setCurrentIndex(self._stage)
        self._refresh()

    def update_state(self, sim, consumed_command: Command | None, pending_command: Command) -> None:
        """Display live state without confusing pending edits with the last tick."""
        self._sim = sim
        self._latest_diagnostic = getattr(sim, "last_diagnostic", None)
        self._latest_consumed_command = _copy_command(consumed_command)
        self._pending_command = _copy_command(pending_command) or Command()
        # Once captured, waveform phase/command and torque must refer to the
        # same tick. Pending control edits stay in the separate preview field.
        if self._diagnostic is None:
            self.waveform.set_state(self._pending_command, float(getattr(sim, "time", 0.0)),sim.config.max_speed)
        preview = self._pending_command.raw_targets(float(getattr(sim, "time", 0.0)))
        actions = ("holding", "holding")
        latest = self._latest_diagnostic
        if self._diagnostic is None or self.live_check.isChecked():
            self.contact_view.set_contacts(latest.contacts if latest is not None else (),f'Latest step: {latest.time_before_s:.6f} s' if latest else 'No completed step')
        if latest is not None:
            before = latest.rotor_speeds_before_rad_s
            after = latest.rotor_speeds_after_rad_s
            magnitude_change = tuple(abs(after[i])-abs(before[i]) for i in range(2))
            actions = tuple("speeding up" if change > 1e-12
                            else "slowing" if change < -1e-12
                            else "holding" for change in magnitude_change)
            preview = after
        self.rotor_bars.set_state(preview, actions, getattr(sim.config, "max_speed", 450.0))
        self._refresh_pending()
        if self.live_check.isChecked():
            self.capture_current()
        elif self._diagnostic is None:
            self._refresh()

    def capture_current(self) -> bool:
        """Freeze the latest completed tick, command, config, and displayed state.

        Returns ``False`` when there is no completed tick to capture.
        """
        if self._latest_diagnostic is None or self._sim is None:
            self._refresh()
            return False
        self._diagnostic = self._latest_diagnostic
        self._consumed_command = _copy_command(self._latest_consumed_command)
        self._captured_config = copy.deepcopy(self._sim.config)
        self._captured_state = {
            "velocity": np.asarray(self._sim.velocity, dtype=float).copy(),
            "rotation": np.asarray(self._sim.rotation, dtype=float).copy(),
            "position": np.asarray(self._sim.position, dtype=float).copy(),
        }
        self._captured_impulses = (self.recorded_ticks, self.body_forward_impulse_ns,
                                   self.world_impulse_ns.copy())
        self.contact_view.set_contacts(self._diagnostic.contacts,f'Captured step: {self._diagnostic.time_before_s:.6f} s')
        self.torque_view.set_tick(self._diagnostic,self._captured_config)
        if self._consumed_command is not None:
            self.waveform.set_state(self._consumed_command,self._diagnostic.time_before_s,self._captured_config.max_speed)
        self._refresh()
        return True

    def record_tick(self, diagnostic) -> None:
        """Accumulate actual contact impulse from one completed immutable tick."""
        rotation = np.asarray(diagnostic.rotation_before, dtype=float).reshape(3, 3)
        force = np.asarray(diagnostic.contact_force_n, dtype=float)
        impulse = force*float(diagnostic.dt_s)
        self.world_impulse_ns += impulse
        # This is a sum of projections onto each tick's pre-step body +X.  Since
        # that frame can turn, it is not a world impulse, mΔv, or distance.
        self.body_forward_impulse_ns += float(np.dot(force, rotation[:, 0]))*float(diagnostic.dt_s)
        self.recorded_ticks += 1
        self._latest_diagnostic = diagnostic
        # Capture only from update_state, after the parent supplies the matching
        # consumed command and completed state; never within the physics loop.

    def reset_history(self) -> None:
        self.body_forward_impulse_ns = 0.0
        self.world_impulse_ns[:] = 0.0
        self.recorded_ticks = 0
        self._diagnostic = None
        self._latest_diagnostic = None
        self._consumed_command = None
        self._captured_config = None
        self._captured_state = None
        self._captured_impulses = None
        self.contact_view.set_contacts(())
        self.torque_view.set_tick(None,None)
        self._refresh()

    def _request_quarter_cycle(self) -> None:
        frequency = float(self._pending_command.frequency)
        seconds = 0.25/frequency if math.isfinite(frequency) and frequency > 0 else 0.25
        self.step_requested.emit(seconds)

    def _inspect(self) -> None:
        self.inspect_requested.emit(("control", "control", "motor", "rotation", "contacts", "translation")[self._stage])

    def _command_formula(self, command: Command, t: float) -> tuple[str, tuple[float, float]]:
        targets = command.raw_targets(t)
        if not command.enabled:
            return "motors disabled ⇒ (L, R) = (0, 0) rad/s", targets
        if command.mode == "Manual rotors":
            return f"manual (L, R) = ({command.left:+.3f}, {command.right:+.3f}) rad/s", targets
        if command.mode == "Constant counterspin":
            return f"(L, R) = (+{command.spin:.3f}, −{command.spin:.3f}) + {command.bias:+.3f} = {_pair(targets)} rad/s", targets
        phase = 2*math.pi*command.frequency*t
        wave = command.amplitude*math.sin(phase)
        return (f"w={command.amplitude:.3f} sin(2π·{command.frequency:.3f} Hz·{t:.4f} s)={wave:+.3f} rad/s; "
                f"(L,R)=(+{command.spin:.3f}+w+{command.bias:+.3f}, −{command.spin:.3f}+w+{command.bias:+.3f})={_pair(targets)} rad/s"), targets

    def _refresh_pending(self) -> None:
        t = float(getattr(self._sim, "time", 0.0)) if self._sim is not None else 0.0
        formula, _targets = self._command_formula(self._pending_command, t)
        self.pending_label.setText("Pending next step: " + formula)
        self.pending_label.setVisible(self.math_toggle.isChecked())

    def _refresh(self) -> None:
        d, sim = self._diagnostic, self._sim
        config = self._captured_config or (sim.config if sim is not None else None)
        state = self._captured_state
        if d is not None:
            command = self._consumed_command or self._pending_command
            t = float(d.time_before_s)
            unavailable = " · consumed command unavailable; using recorded targets" if self._consumed_command is None else ""
            provenance = (f"FROZEN previous step: t0={t:.6f} -> {t+d.dt_s:.6f} s "
                          f"(dt={d.dt_s:.6f} s){unavailable}")
        else:
            command = self._pending_command; t = float(getattr(sim, "time", 0.0)) if sim is not None else 0.0
            provenance = "PREVIEW ONLY: no completed step has been captured"
        self._refresh_pending()
        formula, targets = self._command_formula(command, t)
        if d is not None and self._consumed_command is None:
            targets = tuple(d.targets_rad_s)
            formula = f"recorded consumed targets (L, R) = {_pair(targets)} rad/s"
        intro = STAGES[self._stage][1]
        inspectable = self._stage > 0
        self.inspect_button.setVisible(inspectable)

        if self._stage == 0:
            intuition = ("Counterspin starts L positive and R negative. In the alternating gait, adding positive modulation makes left faster in magnitude but right less negative/slower. "
                         "The default demonstration may move mostly toward initial −X while chassis +X is the green forward axis; no direction is guaranteed.")
            one_formula = formula
        elif self._stage == 1:
            intuition = "The yellow cursor marks phase; quarter-cycle guides show where the sine reaches zero or an extreme. The pair is a speed request, not a position request."
            one_formula = formula
        elif self._stage == 2:
            intuition = "The servo sees target minus the rotor speed at the start of the previous step, then clips the result to the motor limit."
            if d is None or config is None:
                one_formula = f"tau = clip(ks(target-speed), +/-taumax); target={_pair(targets)} rad/s"
            else:
                err=np.asarray(d.requested_rad_s)-np.asarray(d.rotor_speeds_before_rad_s)
                one_formula=(f"tau=clip({config.servo_gain:.4f} N.m/(rad/s) * {_pair(err)} rad/s, +/-{config.max_torque:.4f} N.m) "
                             f"= {_pair(d.motor_torques_nm)} N·m")
        elif self._stage == 3:
            intuition = "The rotors push back on the chassis. Floor torque and the gyroscopic term join that reaction to determine rocking."
            if d is None or config is None:
                one_formula = "alpha = inv(J) * (contact + reaction + gyro)  [rad/s^2]"
            else:
                total=np.asarray(d.contact_torque_body_nm)+np.asarray(d.motor_reaction_nm)+np.asarray(d.gyroscopic_nm)
                one_formula=f"α = J⁻¹({_pair3(total)} N·m) = {_pair3(d.angular_acceleration_rad_s2)} rad/s²"
        elif self._stage == 4:
            intuition = "Yellow dots are feet carrying vertical support force; gray feet are unloaded. Dashed ghosts project each recorded force onto world XY. Cyan sums those horizontal pushes. Vertical support is not forward thrust."
            if d is None:
                one_formula = "previous-step contact impulse = Σ Fcontact Δt  [N·s]"
            else:
                horizontal = np.asarray(d.contact_force_n)[:2]*d.dt_s
                one_formula=f"previous-step horizontal impulse = {_pair(d.contact_force_n[:2])} N × {d.dt_s:.6g} s = {_pair(horizontal)} N·s"
                rows=[]
                for c in d.contacts:
                    rows.append(f"foot {c.foot+1}: N={c.normal_n:.3f} N, friction={_pair(c.friction_n)} N")
                intuition += "<br>"+"; ".join(rows)
        else:
            intuition = ("Forward speed is measured along the chassis' current +X. World x/y displacement stays in fixed axes. The accumulated body-forward contact impulse projects each tick in its own changing frame: it is not mΔv and not distance.")
            if state is None:
                one_formula = "vforward = vworld . R[:,0]  [m/s]"
            else:
                forward=float(np.dot(state["velocity"], state["rotation"][:,0]))
                pos=state["position"]
                one_formula=(f"vforward={_pair3(state['velocity'])} m/s . R[:,0] = {forward:+.4f} m/s; "
                             f"world displacement (x,y)=({_pair(pos[:2])}) m")
            count, body_impulse, world_impulse = (self._captured_impulses if self._captured_impulses is not None
                                                   else (self.recorded_ticks, self.body_forward_impulse_ns,
                                                         self.world_impulse_ns))
            intuition += (f"<br><b>Since reset at capture ({count} ticks):</b> changing-body-forward contact impulse "
                          f"{body_impulse:+.6f} N·s; world impulse {_pair3(world_impulse)} N·s.")

        # Separate stable fields prevent a changing rich-text document from
        # reflowing the equation as telemetry arrives.
        self.provenance_label.setText(provenance)
        plain_intuition = intuition.replace("<br>", "\n").replace("<b>", "").replace("</b>", "")
        self.detail.setText(f"{self._stage+1}. {STAGES[self._stage][0]}\n\n{intro}\n\n{plain_intuition}")
        self.equation_label.setText(one_formula)


def _pair3(values) -> str:
    values = tuple(float(x) for x in values)
    return "(" + ", ".join(f"{x:+.3f}" for x in values) + ")"
