"""PySide desktop viewer; all motion comes from the physics core."""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from collections import deque
from dataclasses import asdict
from pathlib import Path

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QAction, QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
    QFileDialog, QFormLayout, QGroupBox, QHBoxLayout, QLabel, QMainWindow,
    QMessageBox, QPushButton, QMenu, QScrollArea, QSplitter, QTabWidget, QToolBar, QVBoxLayout, QWidget,
)

from .control import Command
from .physics import Config, Simulator
from .explain import KEYS, explanation


STYLE = """
QWidget { background:#111925; color:#d6e0ec; font:13px 'Inter','Helvetica Neue'; }
QMainWindow { background:#0b1018; }
QToolBar { spacing:8px; padding:8px; border-bottom:1px solid #26364a; }
QPushButton { background:#24354a; border:1px solid #38516d; border-radius:4px; padding:7px 12px; font-weight:600; }
QPushButton:hover { background:#2d4661; }
QPushButton:checked { background:#9c3f66; border-color:#ef5da8; }
QPushButton:disabled { color:#65758a; }
QComboBox,QDoubleSpinBox { background:#0c131d; border:1px solid #2d4057; border-radius:3px; padding:5px; color:#b9e7f7; }
QTabWidget::pane { border:1px solid #26364a; }
QTabBar::tab { padding:8px 12px; background:#172333; }
QTabBar::tab:selected { background:#29435e; color:#76d5f7; }
QLabel#section { color:#76d5f7; font-weight:700; letter-spacing:1px; }
QLabel#detail { background:#0e1520; padding:12px; }
QLabel#state_badge { color:#76d5f7; font-weight:700; padding-left:10px; }
QMenuBar,QMenu { background:#111925; }
QMenu::item:selected { background:#29435e; }
QToolTip { background:#172333; color:#d6e0ec; border:1px solid #38516d; }
"""


class Arena(QWidget):
    vector_clicked = Signal(str)

    def __init__(self, sim):
        super().__init__()
        self.sim = sim
        self.trail = deque(maxlen=1800)
        self.top = False
        self.scale = 900.0
        self.show_vectors = True
        self.show_trail = True
        self._vector_hits = []
        self.setMouseTracking(True)
        self.setMinimumSize(500, 280)

    def project(self, point):
        x, y, z = point
        if self.top:
            u, v = -y, -x
        else:
            u, v = .8*x - .6*y, .32*x + .43*y - z
        return QPointF(self.width()/2 + u*self.scale, self.height()*.57 + v*self.scale)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#101923"))
        sim = self.sim
        self._vector_hits = []
        # Follow in x/y only; vertical displacement remains visible.
        origin = np.array([sim.position[0], sim.position[1], 0.0])
        def point(v):
            return self.project(np.asarray(v) - origin)
        def line(a, b, color, width=1):
            p.setPen(QPen(QColor(color), width))
            p.drawLine(point(a), point(b))
        for axis in (0, 1):
            center = math.floor(origin[axis] / .1)
            for n in range(center - 15, center + 16):
                a, b = origin.copy(), origin.copy()
                a[axis] = b[axis] = n*.1
                a[1-axis] -= 1.5
                b[1-axis] += 1.5
                line(a, b, "#253546")
        if self.show_trail and len(self.trail) > 1:
            p.setPen(QPen(QColor("#36bcb3"), 2))
            p.drawPolyline(QPolygonF([point(v) for v in self.trail]))
        # Visual body outline only; actual collision primitives are the feet.
        body = [(-.09,-.11,0),(.09,-.11,0),(.09,.11,0),(-.09,.11,0)]
        verts = [sim.position + sim.rotation @ np.array(v) for v in body]
        p.setPen(QPen(QColor("#aebed0"), 2))
        p.setBrush(QColor("#394e65"))
        p.drawPolygon(QPolygonF([point(v) for v in verts]))
        for i, foot in enumerate(sim.feet_body):
            world = sim.position + sim.rotation @ foot
            line(sim.position + sim.rotation @ (foot*np.array([1,1,0])), world, "#8499ab", 3)
            force = sim.contact_forces[i]
            loaded = force[2] > .01
            p.setBrush(QColor("#ffd166" if loaded else "#536578"))
            p.drawEllipse(point(world), 5, 5)
            if loaded and self.show_vectors:
                start, end = point(world), point(world + force*.008)
                line(world, world + force*.008, "#ffd166", 2)
                self._vector_hits.append(("contacts", start, end))
        for i, center in enumerate(sim.rotor_centers_body):
            c = sim.position + sim.rotation @ center
            axis = sim.axes_body[i]
            u = np.array([1.,0.,0.])
            u -= axis*np.dot(axis,u)
            u /= np.linalg.norm(u)
            v = np.cross(axis,u)
            color = "#ff796d" if i == 0 else "#74b9ff"
            ring = [point(c + sim.rotation @ (.105*(math.cos(t)*u + math.sin(t)*v))) for t in np.linspace(0,2*math.pi,49)]
            p.setPen(QPen(QColor(color), 1))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawPolyline(QPolygonF(ring))
            a = sim.rotor_angles[i]
            d = sim.rotation @ (.105*(math.cos(a)*u + math.sin(a)*v))
            line(c-d, c+d, color, 6)
            line(c, c + sim.rotation @ axis*.08, color, 2)
        forward = sim.position + sim.rotation @ np.array([.15,0,0])
        line(sim.position, forward, "#80ed99", 3)
        if self.show_vectors:
            velocity_end = sim.position + sim.velocity*.08
            line(sim.position, velocity_end, "#76d5f7", 3)
            self._vector_hits.append(("translation", point(sim.position), point(velocity_end)))
        p.setPen(QColor("#dce5ef"))
        p.drawText(18, 27, "DROOPY  /  TWIN-ROTOR DYNAMICS")
        p.drawText(18, 49, "Grid 10 cm  ·  yellow: contact forces  ·  green: forward (+X)")
        p.drawText(18, self.height()-38, "Camera follows robot. Ring = rotor sweep; blades may alias at display rate.")
        p.drawText(18, self.height()-18, "Exploratory model · not measured Droopy geometry · no weapon impacts")
        p.end()

    @staticmethod
    def _distance(pos, a, b):
        ab = b-a
        length = ab.x()*ab.x()+ab.y()*ab.y()
        if not length:
            return math.hypot(pos.x()-a.x(), pos.y()-a.y())
        t=max(0.,min(1.,((pos.x()-a.x())*ab.x()+(pos.y()-a.y())*ab.y())/length))
        return math.hypot(pos.x()-a.x()-t*ab.x(),pos.y()-a.y()-t*ab.y())

    def mouseMoveEvent(self, event):
        found = [(self._distance(event.position(),a,b),key) for key,a,b in self._vector_hits]
        key = min(found)[1] if found and min(found)[0] < 12 and self.show_vectors else None
        self.setCursor(Qt.CursorShape.PointingHandCursor if key else Qt.CursorShape.ArrowCursor)
        self.setToolTip({"contacts":"Floor reaction: spring/damper normal force and traction-limited friction. Click for this tick’s calculation.",
                         "translation":"Chassis velocity from contact forces and gravity. Click for F/m and integration."}.get(key,""))

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self.show_vectors:
            found = [(self._distance(event.position(),a,b),key) for key,a,b in self._vector_hits]
            if found and min(found)[0] < 12:
                self.vector_clicked.emit(min(found)[1])


class Trace(QWidget):
    CHANNELS = {
        "target_l": ("raw L", "#ff9f8f"), "target_r": ("raw R", "#91c7ff"),
        "requested_l": ("clamped L", "#ffc9b8"), "requested_r": ("clamped R", "#c0e2ff"),
        "speed_l": ("actual L", "#ff4f3d"), "speed_r": ("actual R", "#3498db"),
        "error_l": ("error L", "#ffd166"), "error_r": ("error R", "#80ed99"),
        "torque_l": ("torque L", "#ef5da8"), "torque_r": ("torque R", "#b794f4"),
    }

    def __init__(self):
        super().__init__()
        self.samples = deque(maxlen=20000)
        self.window_s = 5.0
        self.max_speed = 450.
        self.max_torque = .8
        self.visible = {key: key in ("requested_l","requested_r","speed_l","speed_r") for key in self.CHANNELS}
        self.setMinimumHeight(245)
        self.setMouseTracking(True)

    def displayed(self):
        if not self.samples:
            return []
        cutoff = self.samples[-1]['t'] - self.window_s
        return [s for s in self.samples if s['t'] >= cutoff]

    def mouseMoveEvent(self, event):
        samples = self.displayed()
        if not samples:
            return
        fraction = max(0.,min(1.,(event.position().x()-55)/max(1,self.width()-70)))
        start=max(0.,samples[-1]['t']-self.window_s)
        end=max(self.window_s,samples[-1]['t'])
        selected_time=start+fraction*(end-start)
        index=int(np.searchsorted([s['t'] for s in samples],selected_time))
        sample=samples[min(index,len(samples)-1)]
        lines = [f"Consumed at t₀={sample['t']:.6f} s; actual speed shown after that tick"]
        for key,(label,_) in self.CHANNELS.items():
            if self.visible[key]:
                unit = 'N·m' if key.startswith('torque') else 'rad/s'
                lines.append(f"{label}: {sample[key]:+.4f} {unit}")
        self.setToolTip('\n'.join(lines))

    def paintEvent(self, event):
        p=QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing); p.fillRect(self.rect(),QColor("#101923"))
        active=[key for key in self.CHANNELS if self.visible[key]]
        p.setPen(QColor("#c6d3e0")); p.drawText(12,19,"MOTOR CONTROL HISTORY  ·  hover for values  ·  target / actual-after-tick")
        x,y=12,37
        for key in active:
            label=self.CHANNELS[key][0]
            width=p.fontMetrics().horizontalAdvance(label)+18
            if x+width>self.width()-12: x,y=12,y+18
            p.setPen(QColor(self.CHANNELS[key][1])); p.drawText(x,y,label)
            x += width
        groups=[([k for k in active if not k.startswith(('torque','error'))],"speed rad/s"),
                ([k for k in active if k.startswith('error')],"error rad/s"),
                ([k for k in active if k.startswith("torque")],"N·m")]
        groups=[g for g in groups if g[0]]
        samples=self.displayed()
        if len(samples)>1 and groups:
            chart_top=y+9
            lane_h=(self.height()-chart_top-24)/len(groups)
            end=samples[-1]['t']
            start=max(0.,end-self.window_s)
            end=max(self.window_s,end)
            span=max(1e-12,end-start)
            stride=max(1,len(samples)//max(1,self.width()*2))
            drawing=samples[::stride]
            if drawing[-1] is not samples[-1]: drawing.append(samples[-1])
            for lane,(keys,unit) in enumerate(groups):
                top=chart_top+lane*lane_h; mid=top+12+lane_h/2
                p.setPen(QColor("#344659")); p.drawLine(55,int(mid),self.width()-15,int(mid))
                limit=self.max_torque if unit=='N·m' else (0. if unit=='error rad/s' else self.max_speed)
                scale=max(1e-6,limit,max(abs(s[k]) for s in samples for k in keys))*1.08
                p.setPen(QColor("#7f91a8")); p.drawText(12,int(top+12),f"±{scale:.3g} {unit}")
                pen=QPen(QColor('#5b6274'),1,Qt.PenStyle.DashLine); p.setPen(pen)
                if limit:
                    for sign in (-1,1):
                        yy=mid-sign*limit/scale*(lane_h-35)/2
                        p.drawLine(55,int(yy),self.width()-15,int(yy))
                for key in keys:
                    points=[QPointF(55+(s['t']-start)/span*(self.width()-70),mid-s[key]/scale*(lane_h-35)/2) for s in drawing]
                    pen=QPen(QColor(self.CHANNELS[key][1]),2)
                    if key.startswith('target'): pen.setStyle(Qt.PenStyle.DashLine)
                    p.setPen(pen); p.drawPolyline(QPolygonF(points))
            p.setPen(QColor("#7f91a8")); p.drawText(55,self.height()-7,f"{start:.3f} s")
            p.drawText(self.width()-90,self.height()-7,f"{end:.3f} s")
        p.end()


class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Droopy Engineering Simulator")
        self.resize(1280,860)
        self.sim = Simulator()
        self.running = False
        self.accumulator = 0.0
        self.rate_wall_start=time.monotonic(); self.rate_sim_start=0.; self.actual_rate=0.
        self.record_file = None
        self.record_writer = None
        self.record_path = None
        self.record_meta = None
        self.last_consumed_command = None
        self.last_batch_count = 0
        self.arena = Arena(self.sim)
        self.trace = Trace()
        self.inputs = {}
        self.settings = {}
        self._build()
        self.last_wall = time.monotonic()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(16)
        self.refresh()

    def number(self, form, name, low, high, value, step=1., decimals=2):
        box = QDoubleSpinBox()
        box.setRange(low,high)
        box.setDecimals(decimals)
        box.setSingleStep(step)
        box.setValue(value)
        form.addRow(name,box)
        return box

    def _build(self):
        self.setStyleSheet(STYLE)
        self.setMinimumSize(1080,720)
        plot_menu=self.menuBar().addMenu("Plot channels")
        self.plot_actions={}
        for title,keys in [('Speed tracking',('requested_l','requested_r','speed_l','speed_r')),
                           ('Left servo',('target_l','requested_l','speed_l','error_l','torque_l')),
                           ('Right servo',('target_r','requested_r','speed_r','error_r','torque_r')),
                           ('Torque only',('torque_l','torque_r'))]:
            preset=plot_menu.addAction(title)
            preset.triggered.connect(lambda checked=False,selection=keys: self.plot_preset(selection))
        plot_menu.addSeparator()
        for key,(label,_) in Trace.CHANNELS.items():
            action=QAction(label,self); action.setCheckable(True); action.setChecked(self.trace.visible[key])
            action.toggled.connect(lambda checked,k=key: self.set_plot_channel(k,checked)); plot_menu.addAction(action)
            self.plot_actions[key]=action
        span_menu=plot_menu.addMenu('History window')
        for seconds in (1,5,10):
            item=span_menu.addAction(f'{seconds} seconds')
            item.triggered.connect(lambda checked=False,s=seconds: self.set_plot_window(s))
        toolbar = QToolBar("Simulation"); toolbar.setMovable(False); self.addToolBar(toolbar)
        self.run_button = QPushButton("▶  START"); self.run_button.setCheckable(True); self.run_button.clicked.connect(self.toggle_run)
        self.step_button = QPushButton("STEP"); self.step_button.setToolTip("Advance exactly one fixed physics tick."); self.step_button.clicked.connect(lambda: self.advance(1))
        self.reset_button = QPushButton("RESET"); self.reset_button.clicked.connect(self.reset)
        self.record = QPushButton("RECORD CSV…"); self.record.clicked.connect(self.toggle_record)
        self.settings_button = QPushButton("⚙ SETTINGS"); self.settings_button.clicked.connect(self.show_settings)
        for w in (self.run_button,self.step_button,self.reset_button,self.record,self.settings_button): toolbar.addWidget(w)
        toolbar.addSeparator(); self.time_label=QLabel("t = 0.000 s"); toolbar.addWidget(self.time_label)
        self.state_label=QLabel("SIM PAUSED"); self.state_label.setObjectName("state_badge"); toolbar.addWidget(self.state_label)

        root = QSplitter(); self.setCentralWidget(root)
        self.side_tabs=QTabWidget()
        side_scroll=QScrollArea(); side_scroll.setWidgetResizable(True)
        left=QWidget(); layout=QVBoxLayout(left); side_scroll.setWidget(left)
        self.side_tabs.addTab(side_scroll,'Controls & telemetry')
        heading=QLabel("LIVE CONTROLS + TELEMETRY"); heading.setObjectName("section"); layout.addWidget(heading)
        form=QFormLayout()
        self.enabled=QCheckBox("Enable motors (off = active braking)"); self.enabled.setObjectName("motors_enabled"); form.addRow(self.enabled)
        self.mode=QComboBox(); self.mode.addItems(["Alternating gait","Constant counterspin","Manual rotors"]); form.addRow("Motor command",self.mode)
        for key,label,lo,hi,val,step in [
            ("spin","Base speed [rad/s]",0,450,220,10),("amplitude","Gait amplitude [rad/s]",0,300,180,10),
            ("frequency","Gait frequency [Hz]",.1,10,1,.1),("bias","Signed bias [rad/s]",-300,300,0,10),
            ("left","Manual left [rad/s]",-450,450,220,10),("right","Manual right [rad/s]",-450,450,-220,10)]:
            self.inputs[key]=self.number(form,label,lo,hi,val,step)
        self.speed=QDoubleSpinBox(); self.speed.setRange(.01,4); self.speed.setSingleStep(.1); self.speed.setValue(.5)
        self.speed.setToolTip('Requested playback rate; under heavy load actual simulation runs slower.')
        toolbar.addSeparator(); toolbar.addWidget(QLabel('Speed')); toolbar.addWidget(self.speed)
        realtime=QPushButton('1×'); realtime.clicked.connect(lambda:self.speed.setValue(1)); toolbar.addWidget(realtime)
        layout.addLayout(form)
        note=QLabel("Open-loop rotor targets only — no scripted propulsion or guaranteed steering."); note.setWordWrap(True); layout.addWidget(note)
        self.telemetry=QLabel(); self.telemetry.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse); layout.addWidget(self.telemetry)
        layout.addStretch()
        explain_page=QWidget(); layout=QVBoxLayout(explain_page)
        self.side_tabs.addTab(explain_page,'Physics inspector')
        explain_label=QLabel("LIVE PHYSICS EXPLAINER"); explain_label.setObjectName("section"); layout.addWidget(explain_label)
        flow=QLabel('Open-loop gait → speed target → rotor servo ↔ rotor speed\nTorque → reaction + gyro → attitude → floor contacts\nFloor forces → translation; no chassis-motion feedback.')
        flow.setWordWrap(True); layout.addWidget(flow)
        blocks=QHBoxLayout()
        for key,label in [('control','Setpoint'),('motor','Servo'),('rotation','Gyro'),('contacts','Floor')]:
            block=QPushButton(label); block.clicked.connect(lambda checked=False,k=key:self.open_explanation(k)); blocks.addWidget(block)
        layout.addLayout(blocks)
        self.explain_select=QComboBox(); self.explain_select.addItems([label for _,label in KEYS]); self.explain_select.currentIndexChanged.connect(self.refresh_explanation); layout.addWidget(self.explain_select)
        detail_scroll=QScrollArea(); detail_scroll.setWidgetResizable(True); detail_scroll.setMinimumHeight(280)
        self.detail=QLabel(); self.detail.setObjectName("detail"); self.detail.setWordWrap(True); self.detail.setTextFormat(Qt.TextFormat.RichText); self.detail.setAlignment(Qt.AlignmentFlag.AlignTop); detail_scroll.setWidget(self.detail); layout.addWidget(detail_scroll,1)
        layout.addStretch()

        right=QWidget(); visual=QVBoxLayout(right)
        view=QHBoxLayout(); self.top_check=QCheckBox("Top view"); self.top_check.toggled.connect(self.set_top); view.addWidget(self.top_check)
        self.vectors_check=QCheckBox("Vectors (click arrows)"); self.vectors_check.setChecked(True); self.vectors_check.toggled.connect(self.set_vectors); view.addWidget(self.vectors_check)
        self.trail_check=QCheckBox("Trail"); self.trail_check.setChecked(True); self.trail_check.toggled.connect(self.set_trail); view.addWidget(self.trail_check)
        self.zoom=QComboBox(); self.zoom.addItems(["Close","Medium","Wide"]); self.zoom.currentIndexChanged.connect(self.set_zoom); view.addWidget(self.zoom); view.addStretch(); visual.addLayout(view)
        plot_controls=QHBoxLayout()
        plot_button=QPushButton('Plot channels ▾'); plot_button.setMenu(plot_menu); plot_controls.addWidget(plot_button)
        self.plot_window_label=QLabel('Window: 5 s · ≤20,000 ticks'); plot_controls.addWidget(self.plot_window_label); plot_controls.addStretch()
        visual.addLayout(plot_controls)
        visual.addWidget(self.arena,1); visual.addWidget(self.trace)
        self.arena.vector_clicked.connect(self.open_explanation)
        self.view_splitter=QSplitter(Qt.Orientation.Vertical)
        visual.removeWidget(self.arena); visual.removeWidget(self.trace)
        self.view_splitter.addWidget(self.arena); self.view_splitter.addWidget(self.trace)
        self.view_splitter.setSizes([430,280]); visual.addWidget(self.view_splitter,1)
        root.addWidget(self.side_tabs); root.addWidget(right); root.setSizes([430,850])
        self.settings_dialog=self._settings_dialog()
        self.statusBar().showMessage("Paused. Enable motors, then Start. Settings reset the experiment.")
        self.mode.currentTextChanged.connect(self.mode_changed); self.mode_changed(self.mode.currentText())
        self.mode.currentTextChanged.connect(lambda _: self.refresh())
        self.enabled.toggled.connect(lambda _: self.refresh())
        for box in self.inputs.values(): box.valueChanged.connect(lambda _: self.refresh())

    def _settings_dialog(self):
        dialog=QDialog(self); dialog.setWindowTitle("Physical model settings"); dialog.resize(470,420)
        outer=QVBoxLayout(dialog); scroll=QScrollArea(); scroll.setWidgetResizable(True); body=QWidget(); form=QFormLayout(body)
        specs=(
            ("dt","Fixed tick [s]",.00001,.01,.0001),("mass","Mass [kg]",.01,100,.1),
            ("tilt_deg","Outward rotor tilt [°]",0,89,1),("friction","Floor friction μ",0,10,.05),
            ("rotor_inertia","Rotor inertia [kg·m²]",.000001,1,.00005),("max_torque","Motor torque limit [N·m]",.0001,100,.05),
            ("max_speed","Max rotor speed [rad/s]",1,10000,10),("gravity","Gravity [m/s²]",0,100,.1),
            ("body_inertia_x","Body inertia X [kg·m²]",.000001,10,.001),("body_inertia_y","Body inertia Y [kg·m²]",.000001,10,.001),
            ("body_inertia_z","Body inertia Z [kg·m²]",.000001,10,.001),("rotor_offset_y","Rotor offset Y [m]",.0001,10,.01),
            ("rotor_height","Rotor height [m]",-10,10,.01),("foot_half_length","Foot half-length [m]",.0001,10,.005),
            ("foot_half_width","Foot half-width [m]",.0001,10,.005),("foot_drop","Foot drop [m]",.0001,10,.005),
            ("contact_stiffness","Contact stiffness [N/m]",1,1e7,100),("contact_damping","Contact damping [N·s/m]",.001,1e5,1),
            ("friction_velocity","Friction smoothing [m/s]",.000001,10,.001),("servo_gain","Servo gain [N·m/(rad/s)]",.000001,10,.001))
        for key,label,lo,hi,step in specs:
            self.settings[key]=self.number(form,label,lo,hi,getattr(self.sim.config,key),step,6)
        scroll.setWidget(body); outer.addWidget(scroll)
        buttons=QDialogButtonBox(QDialogButtonBox.StandardButton.Apply|QDialogButtonBox.StandardButton.Reset|QDialogButtonBox.StandardButton.Close)
        buttons.button(QDialogButtonBox.StandardButton.Apply).setText("APPLY & RESET")
        buttons.button(QDialogButtonBox.StandardButton.Reset).setText("RESTORE DEFAULTS")
        buttons.button(QDialogButtonBox.StandardButton.Apply).clicked.connect(self.apply_config)
        buttons.button(QDialogButtonBox.StandardButton.Reset).clicked.connect(self.reset_settings)
        buttons.rejected.connect(dialog.hide); outer.addWidget(buttons)
        return dialog

    def show_settings(self):
        self.settings_dialog.show(); self.settings_dialog.raise_()

    def reset_settings(self):
        defaults=Config()
        for key,widget in self.settings.items(): widget.setValue(getattr(defaults,key))

    def mode_changed(self, mode):
        manual = mode == "Manual rotors"
        for key, widget in self.inputs.items():
            widget.setEnabled((key in ("left","right")) == manual)
        if mode == "Constant counterspin":
            self.inputs["amplitude"].setEnabled(False)
            self.inputs["frequency"].setEnabled(False)

    def command(self):
        return Command(enabled=self.enabled.isChecked(),mode=self.mode.currentText(),**{k:w.value() for k,w in self.inputs.items()})

    def toggle_run(self):
        self.running = not self.running
        self.last_wall = time.monotonic()
        self.rate_wall_start=self.last_wall; self.rate_sim_start=self.sim.time
        self.run_button.setChecked(self.running)
        self.run_button.setText("❚❚  PAUSE" if self.running else "▶  START")
        self.state_label.setText("SIM RUNNING" if self.running else "SIM PAUSED")
        self.step_button.setEnabled(not self.running)

    def tick(self):
        now = time.monotonic(); elapsed = min(.1,now-self.last_wall); self.last_wall = now
        if self.running:
            self.accumulator += elapsed*self.speed.value()
            count = min(100,int(self.accumulator/self.sim.config.dt))
            self.accumulator -= count*self.sim.config.dt
            # Drop backlog under load rather than freezing controls.
            self.accumulator = min(self.accumulator,.1)
            if count: self.advance(count)
            interval=now-self.rate_wall_start
            if interval >= .5:
                self.actual_rate=(self.sim.time-self.rate_sim_start)/interval
                self.rate_wall_start=now; self.rate_sim_start=self.sim.time
                self.state_label.setText(f'SIM RUNNING · {self.actual_rate:.2f}× actual')

    def advance(self, count):
        cmd = self.command(); self.last_batch_count=count
        try:
            for _ in range(count):
                t0=self.sim.time; self.sim.step(cmd.raw_targets(t0))
                self.last_consumed_command=Command(**asdict(cmd)); d=self.sim.last_diagnostic
                sample={"t":t0,"target_l":d.targets_rad_s[0],"target_r":d.targets_rad_s[1],
                        "requested_l":d.requested_rad_s[0],"requested_r":d.requested_rad_s[1],
                        "speed_l":d.rotor_speeds_after_rad_s[0],"speed_r":d.rotor_speeds_after_rad_s[1],
                        "error_l":d.requested_rad_s[0]-d.rotor_speeds_before_rad_s[0],
                        "error_r":d.requested_rad_s[1]-d.rotor_speeds_before_rad_s[1],
                        "torque_l":d.motor_torques_nm[0],"torque_r":d.motor_torques_nm[1]}
                self.trace.samples.append(sample)
                if self.record_meta is not None:
                    event={"consumed_at_t0":t0,"command":asdict(cmd)}; events=self.record_meta["commands"]
                    if not events or events[-1]["command"] != event["command"]: events.append(event)
                if self.record_writer:
                    row=self.sim.snapshot(); row.update({"target_left_rad_s":sample["target_l"],"target_right_rad_s":sample["target_r"],
                        "consumed_at_s":t0,"requested_left_rad_s":sample['requested_l'],"requested_right_rad_s":sample['requested_r'],
                        "actual_left_rad_s":sample["speed_l"],"actual_right_rad_s":sample["speed_r"],
                        "error_left_rad_s":sample["error_l"],"error_right_rad_s":sample["error_r"],
                        "torque_left_nm":sample["torque_l"],"torque_right_nm":sample["torque_r"]})
                    self.record_writer.writerow(row)
        except (ValueError,FloatingPointError,RuntimeError,OSError) as exc:
            if self.running: self.toggle_run()
            if isinstance(exc,OSError): self.finish_record(); self.statusBar().showMessage(f"Recording stopped: {exc}")
            else: QMessageBox.critical(self,"Simulation stopped",str(exc))
            return
        self.arena.trail.append([self.sim.position[0],self.sim.position[1],0.0]); self.refresh()

    def refresh(self):
        d = self.sim.snapshot()
        pending = self.command().targets(self.sim.time,self.sim.config.max_speed)
        diag=self.sim.last_diagnostic
        if diag is None:
            consumed="Consumed tick  —"
        else:
            consumed=(f"Consumed {diag.time_before_s:.6f}→{diag.time_before_s+diag.dt_s:.6f} s\n"
                      f"  raw {diag.targets_rad_s[0]:+.1f} / {diag.targets_rad_s[1]:+.1f} rad/s\n"
                      f"  clamped {diag.requested_rad_s[0]:+.1f} / {diag.requested_rad_s[1]:+.1f} rad/s\n"
                      f"  speed {diag.rotor_speeds_before_rad_s[0]:+.1f} / {diag.rotor_speeds_before_rad_s[1]:+.1f} rad/s\n"
                      f"  torque {diag.motor_torques_nm[0]:+.3f} / {diag.motor_torques_nm[1]:+.3f} N·m")
        self.time_label.setText(f"t = {self.sim.time:.3f} s")
        self.telemetry.setText(
            f"Time  {d['t']:.3f} s\n"
            f"Position  {d['x']:+.3f}, {d['y']:+.3f}, {d['z']:.3f} m\n"
            f"Tilt  roll {d['roll_deg']:+.1f}°  pitch {d['pitch_deg']:+.1f}°\n"
            f"Yaw  {d['yaw_deg']:+.1f}°   Contacts  {d['contacts']}\n"
            f"Rotors  {d['left_rpm']:+.0f} / {d['right_rpm']:+.0f} RPM\n"
            f"{consumed}\n"
            f"Pending next tick (clamped)  {pending[0]:+.1f} / {pending[1]:+.1f} rad/s\n"
            f"Mechanical energy  {d['energy']:.2f} J")
        if max(abs(d['roll_deg']),abs(d['pitch_deg'])) > 30:
            self.statusBar().showMessage("Large tilt: feet-only collision model is outside its useful upright range. Reset recommended.")
        self.refresh_explanation()
        self.arena.update(); self.trace.update()

    def reset(self):
        if self.running: self.toggle_run()
        self.finish_record()
        self.enabled.setChecked(False)
        self.sim.reset(); self.last_consumed_command=None; self.last_batch_count=0; self.accumulator = 0.
        self.arena.trail.clear(); self.trace.samples.clear(); self.refresh()

    def apply_config(self):
        values = asdict(self.sim.config)
        values.update({k:w.value() for k,w in self.settings.items()})
        try:
            config = Config(**values); config.validate(); candidate = Simulator(config)
        except (ValueError, OverflowError) as exc:
            QMessageBox.warning(self,"Invalid settings",str(exc)); return False
        self.reset()
        self.sim = candidate; self.arena.sim = candidate
        self.trace.max_torque=config.max_torque; self.trace.max_speed=config.max_speed
        self.trace.samples=deque(maxlen=min(200000,max(600,math.ceil(10/config.dt))))
        self.set_plot_window(self.trace.window_s)
        self.refresh()
        self.statusBar().showMessage("Settings applied; experiment reset.")
        return True

    def open_explanation(self, key):
        self.side_tabs.setCurrentIndex(1)
        keys=[item[0] for item in KEYS]
        if key in keys: self.explain_select.setCurrentIndex(keys.index(key))
        self.refresh_explanation()

    def refresh_explanation(self):
        if not hasattr(self,'explain_select'): return
        key=KEYS[self.explain_select.currentIndex()][0]
        diag=self.sim.last_diagnostic
        command=self.last_consumed_command if diag is not None else self.command()
        demand_time=diag.time_before_s if diag is not None else self.sim.time
        pane=self.detail.parentWidget().parentWidget()
        scroll=pane.verticalScrollBar() if isinstance(pane,QScrollArea) else None
        value=scroll.value() if scroll else 0
        self.detail.setText(explanation(key,diag,self.sim.config,command,demand_time))
        if scroll: scroll.setValue(value)

    def plot_preset(self, keys):
        for key,action in self.plot_actions.items(): action.setChecked(key in keys)

    def set_plot_window(self, seconds):
        self.trace.window_s=seconds
        capacity=self.trace.samples.maxlen*self.sim.config.dt
        self.plot_window_label.setText(f'Window: {seconds} s · buffer ≤{capacity:g} s')
        self.trace.update()

    def set_plot_channel(self, key, checked):
        self.trace.visible[key]=bool(checked); self.trace.update()

    def set_vectors(self, value):
        self.arena.show_vectors=bool(value); self.arena.update()

    def set_trail(self, value):
        self.arena.show_trail=bool(value); self.arena.update()

    def set_top(self, value):
        self.arena.top = value; self.arena.update()

    def set_zoom(self, index):
        self.arena.scale = [900.,550.,280.][index]; self.arena.update()

    def toggle_record(self):
        if self.record_file:
            self.finish_record(); return
        name,_ = QFileDialog.getSaveFileName(self,"Record telemetry","droopy.csv","CSV (*.csv)")
        if not name: return
        try:
            self.record_path = Path(name)
            self.record_file = self.record_path.open("w",newline="")
            signals=("consumed_at_s","target_left_rad_s","target_right_rad_s","requested_left_rad_s","requested_right_rad_s","actual_left_rad_s","actual_right_rad_s",
                     "error_left_rad_s","error_right_rad_s","torque_left_nm","torque_right_nm")
            self.record_writer = csv.DictWriter(self.record_file,fieldnames=tuple(self.sim.snapshot().keys())+signals)
            self.record_writer.writeheader()
            self.record_meta = {"schema":"droopy-exploratory-v1","config":asdict(self.sim.config),
                                "start":self.sim.snapshot(),"commands":[],
                                "note":"Telemetry segment, not a complete restart state."}
            self.record.setText("Stop recording")
        except OSError as exc:
            self.finish_record(); QMessageBox.warning(self,"Recording failed",str(exc))

    def finish_record(self):
        if self.record_file:
            try:
                self.record_file.close()
            except OSError as exc:
                self.statusBar().showMessage(f"Recording close failed: {exc}")
            if self.record_meta:
                self.record_meta["end"] = self.sim.snapshot()
                try:
                    self.record_path.with_suffix(".json").write_text(json.dumps(self.record_meta,indent=2)+"\n")
                except OSError as exc:
                    self.statusBar().showMessage(f"Metadata write failed: {exc}")
        self.record_file = self.record_writer = self.record_meta = None
        self.record.setText("Record CSV…")

    def closeEvent(self,event):
        self.timer.stop(); self.finish_record(); event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--smoke",action="store_true",help="advance, render /tmp/droopy-ui.png, and exit")
    args = parser.parse_args()
    app = QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)
    window = Window(); window.show()
    if args.smoke:
        def smoke():
            window.enabled.setChecked(True)
            window.advance(400)
            if not window.grab().save("/tmp/droopy-ui.png"):
                app.exit(1); return
            window.close(); app.quit()
        QTimer.singleShot(50,smoke)
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
