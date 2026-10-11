"""Force integrals and observed straightness, in a fixed world reference."""
import math
from PySide6.QtCore import QPointF,QRectF,Qt
from PySide6.QtGui import QColor,QPainter,QPen,QPolygonF
from PySide6.QtWidgets import QWidget,QVBoxLayout,QHBoxLayout,QLabel,QPushButton,QCheckBox

from .impulse_vectors import ImpulseVectorPanel


class MotionCanvas(QWidget):
    def __init__(self):
        super().__init__();self.data=None
        self.setMinimumSize(340,905)
        self.setAccessibleName('Integrated world forces, cancellation and actual path')

    def set_snapshot(self,data):
        self.data=data
        if data:
            self.setAccessibleDescription(f'Integrated through {data.time_s:.6f} seconds; net impulse {data.net_impulse_ns} Ns; momentum residual {data.residual_ns} Ns; along distance {data.along_displacement_m} m; sideways distance {data.lateral_displacement_m} m')
        else:self.setAccessibleDescription('No force integration samples yet')
        self.update()

    def paintEvent(self,event):
        p=QPainter(self);p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(),QColor('#0e1722'))
        def text(y,s,color='#d6e0ec',x=10):p.setPen(QColor(color));p.drawText(x,y,s)
        d=self.data
        if d is None:
            text(24,'Start or STEP to accumulate actual forces.');p.end();return
        def triple(values):return '('+', '.join(f'{x:+.4f}' for x in values)+')'
        text(22,f'Through t = {d.time_s:.4f} s · {d.ticks:,} physics ticks','#76d5f7')
        text(44,'1  ADD EVERY FORCE × TIME STEP')
        text(65,'J = Σ [Ffloor + (0, 0, −mg)] Δt')
        text(85,'WORLD XYZ impulse [N·s]','#aebed0')
        text(105,'Floor: '+triple(d.contact_impulse_ns),'#ffd166')
        text(125,'Gravity: '+triple(d.gravity_impulse_ns),'#b794f4')
        text(145,'Net: '+triple(d.net_impulse_ns),'#76d5f7')
        text(169,'Check J = m(v − v₀): '+triple(d.momentum_change_kg_mps))
        error=math.sqrt(sum(x*x for x in d.residual_ns))
        text(190,f'Momentum residual: {error:.3e} N·s','#80ed99' if error<1e-7 else '#ff718d')
        text(217,'2  DO THE SIDEWAYS PUSHES CANCEL?')
        text(237,f'Fixed travel axis: ({d.reference_xy[0]:+.2f}, {d.reference_xy[1]:+.2f}) world XY','#aebed0')
        text(255,'Initial body −X; NOT the rotating body frame.','#aebed0')
        # Each pair is a sum of signed impulses, not force magnitudes or distance.
        sums=((d.along_positive_ns,d.along_negative_ns),(d.lateral_positive_ns,d.lateral_negative_ns))
        scale=max(.001,max(abs(v) for pair in sums for v in pair))
        cx=self.width()/2;half=max(30,self.width()/2-25)
        for i,((positive,negative),name) in enumerate(zip(sums,('Along','Sideways'))):
            yy=287+i*54
            text(yy-8,f'{name}: +{positive:.4f} + ({negative:.4f}) = {positive+negative:+.4f} N·s')
            p.setPen(QPen(QColor('#38516d'),1));p.drawLine(QPointF(cx,yy),QPointF(cx,yy+18))
            p.setPen(QPen(QColor('#80ed99'),7));p.drawLine(QPointF(cx,yy+8),QPointF(cx+positive/scale*half,yy+8))
            p.setPen(QPen(QColor('#ff796d'),7));p.drawLine(QPointF(cx,yy+8),QPointF(cx+negative/scale*half,yy+8))
        text(369,'Equal opposite impulses cancel velocity change,','#aebed0')
        text(387,'not velocity itself: a moving robot can coast.','#aebed0')
        text(408,'3  DISTANCE: x₁ = x₀ + v₁ Δt')
        # True geometric aspect ratio, projected in the FIXED reference frame.
        points=[(s[3],s[4]) for s in d.samples]
        if d.samples and d.samples[0][0]<.021: points.insert(0,(0.,0.))
        points.append((d.along_displacement_m,d.lateral_displacement_m))
        bounds=[(0.,0.)]+points
        minx=min(x for x,y in bounds);maxx=max(x for x,y in bounds)
        maxy=max(.002,max(abs(y) for x,y in points))
        box=QRectF(28,444,self.width()-56,137)
        scale_m=min(box.width()/max(.005,maxx-minx),box.height()/(2*maxy))*.9
        origin=QPointF(box.left()-minx*scale_m,box.center().y())
        p.setPen(QPen(QColor('#536578'),1,Qt.PenStyle.DashLine));p.drawLine(QPointF(box.left(),origin.y()),QPointF(box.right(),origin.y()))
        path=[origin+QPointF(x*scale_m,-y*scale_m) for x,y in points]
        p.setPen(QPen(QColor('#76d5f7'),2));p.drawPolyline(QPolygonF(path))
        p.setBrush(QColor('#ffd166'));p.drawEllipse(origin,4,4)
        p.setBrush(QColor('#76d5f7'));p.drawEllipse(path[-1],4,4)
        text(439,'↑ side displacement · → along displacement','#aebed0')
        text(603,f'Along {d.along_displacement_m:+.4f} m · side {d.lateral_displacement_m:+.4f} m')
        text(625,f'Max sideways offset: {d.max_abs_cross_track_m:.4f} m')
        ratio='not moving' if d.straightness_ratio is None else f'{d.straightness_ratio:.3f} (1 = direct path)'
        text(647,'Net distance / path length: '+ratio)
        text(669,f'Speed: along {d.along_velocity_mps:+.4f} / side {d.lateral_velocity_mps:+.4f} m/s','#aebed0')
        text(691,'Path tail bounded; all impulse totals span since reset.','#aebed0')
        text(722,'CUMULATIVE IMPULSE OVER TIME  [N·s]')
        text(742,'Gold = along · cyan = sideways · signed totals','#aebed0')
        samples=d.samples
        if len(samples)>1:
            t0,t1=samples[0][0],samples[-1][0]
            maximum=max(.001,max(abs(s[i]) for s in samples for i in (1,2)))
            mid=810.;height=48.
            p.setPen(QPen(QColor('#38516d'),1));p.drawLine(28,int(mid),self.width()-28,int(mid))
            for i,color in ((1,'#ffd166'),(2,'#76d5f7')):
                curve=[QPointF(28+(s[0]-t0)/max(1e-12,t1-t0)*(self.width()-56),mid-s[i]/maximum*height) for s in samples]
                p.setPen(QPen(QColor(color),2));p.drawPolyline(QPolygonF(curve))
            text(763,f'Scale ±{maximum:.4f} N·s','#aebed0')
            text(880,f'{t0:.3f} s → {t1:.3f} s (retained history)','#aebed0')
        p.end()


class IntegratedMotionView(QWidget):
    def __init__(self):
        super().__init__();self.latest=None;self.frozen=False
        layout=QVBoxLayout(self)
        title=QLabel('WHY THE ROBOT KEEPS MOVING');title.setObjectName('section');layout.addWidget(title)
        note=QLabel('Watch sideways contributions cancel or accumulate. Low sideways impulse alone does not prove a straight path: compare the actual path below. All totals are world-frame integrals since reset. Gravity and floor contact are all external linear forces in this model; motor torque is internal and must not be added as thrust.')
        note.setWordWrap(True);layout.addWidget(note)
        controls=QHBoxLayout()
        self.freeze=QCheckBox('Freeze this view');self.freeze.toggled.connect(self.set_frozen);controls.addWidget(self.freeze)
        capture=QPushButton('Capture latest');capture.clicked.connect(self.capture_latest);controls.addWidget(capture)
        layout.addLayout(controls)
        self.vector_panel=ImpulseVectorPanel();layout.addWidget(self.vector_panel)
        self.state_label=QLabel('Live integration');layout.addWidget(self.state_label)
        self.canvas=MotionCanvas();layout.addWidget(self.canvas)

    def _show_snapshot(self,data):
        self.vector_panel.set_snapshot(data)
        self.canvas.set_snapshot(data)

    def update_snapshot(self,data):
        self.latest=data
        if not self.frozen:self._show_snapshot(data)

    def set_frozen(self,enabled):
        self.frozen=enabled
        self.state_label.setText('Frozen totals/path — new samples still accumulate' if enabled else 'Live integration — every physics tick contributes')
        if not enabled:self._show_snapshot(self.latest)

    def capture_latest(self):
        self.freeze.setChecked(True);self._show_snapshot(self.latest)

    def reset(self):
        self.latest=None;self.freeze.setChecked(False);self._show_snapshot(None)
