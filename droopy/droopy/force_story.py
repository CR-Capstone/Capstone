"""Captured torque-to-translation visualisation. No predicted propulsion."""
from dataclasses import dataclass
import math
import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QWidget


@dataclass(frozen=True)
class ForceStory:
    time: float
    dt: float
    motor: tuple
    gyro: tuple
    floor_torque: tuple
    total_torque: tuple
    force_xy: tuple
    delta_v_xy: tuple
    normal_loads: tuple
    mass: float

    @classmethod
    def from_tick(cls, d, config):
        motor=np.asarray(d.motor_reaction_nm)
        gyro=np.asarray(d.gyroscopic_nm)
        floor=np.asarray(d.contact_torque_body_nm)
        force=np.asarray(d.contact_force_n[:2])
        return cls(d.time_before_s,d.dt_s,tuple(motor),tuple(gyro),tuple(floor),
                   tuple(motor+gyro+floor),tuple(force),tuple(force/config.mass*d.dt_s),
                   tuple(c.normal_n for c in d.contacts),config.mass)


class TorqueMotionView(QWidget):
    """Torque components are body-frame; force arrow is explicitly world XY.

    Torque bars share a per-snapshot scale. Force arrow uses fixed px/N, with
    capped length labelled; arrow sizes cannot be compared between units.
    """
    inspect_requested=Signal(str)

    def __init__(self):
        super().__init__()
        self.story=None
        self.setMinimumHeight(465)
        self.setMinimumWidth(320)
        self.setAccessibleName('Captured motor torque, floor push and velocity change')
        self.setToolTip('Click the torque section for Euler/gyro math; click floor push for translation math. Torque axes are body axes; the force arrow is in world XY.')

    def set_tick(self,diagnostic,config):
        self.story=None if diagnostic is None else ForceStory.from_tick(diagnostic,config)
        if self.story:
            s=self.story
            self.setAccessibleDescription(f'Captured t={s.time:.6f} s; motor reaction body Nm {s.motor}; total torque {s.total_torque}; world floor force N {s.force_xy}; change in world velocity m/s {s.delta_v_xy}')
        else:self.setAccessibleDescription('No captured tick; step then capture. No force prediction from waveform alone.')
        self.update()

    def mousePressEvent(self,event):
        if event.button()==Qt.MouseButton.LeftButton:
            self.inspect_requested.emit('rotation' if event.position().y()<250 else 'translation')

    def paintEvent(self,event):
        p=QPainter(self);p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(),QColor('#0e1722'))
        def text(x,y,value,color='#d6e0ec'):
            p.setPen(QColor(color));p.drawText(x,y,value)
        text(10,20,'TORQUE → ROCKING → FLOOR PUSH', '#76d5f7')
        if self.story is None:
            text(10,48,'Step, then Capture current step.')
            text(10,70,'A speed waveform alone cannot predict push.')
            p.end();return
        s=self.story
        text(10,40,f'Captured {s.time:.6f} s · Δt {s.dt:.6f} s')
        text(10,63,'1  TORQUE ON BODY  [N·m]')
        colors=('#ff796d','#b794f4','#ffd166','#80ed99')
        values=(s.motor,s.gyro,s.floor_torque,s.total_torque)
        names=('motor reaction','gyro term','floor torque','total')
        # Horizontal signed bars reveal cancelling terms as well as resultant.
        scale=max(.01,max(abs(v) for vector in values for v in vector))
        left=68.;width=max(20.,(self.width()-left-12)/3)
        for axis,name in enumerate(('roll X','pitch Y','yaw Z')):
            cx=left+(axis+.5)*width
            text(int(cx-width*.44),83,name,'#aebed0')
            p.setPen(QPen(QColor('#38516d'),1));p.drawLine(QPointF(cx,92),QPointF(cx,196))
            for row,vector in enumerate(values):
                y=99+row*29
                length=vector[axis]/scale*(width*.43)
                p.setPen(QPen(QColor(colors[row]),3));p.drawLine(QPointF(cx,y),QPointF(cx+length,y))
                if abs(length)>2:
                    tip=QPointF(cx+length,y);direction=1 if length>0 else -1
                    p.setBrush(QColor(colors[row]));p.drawPolygon(QPolygonF([tip,tip+QPointF(-direction*5,-3),tip+QPointF(-direction*5,3)]))
                text(int(cx-width*.43),y+14,f'{vector[axis]:+.3f}',colors[row])
        for row,name in enumerate(('motor','gyro','floor','total')):text(8,103+row*29,name,colors[row])
        text(10,222,f'Common bar scale ±{scale:.3f} N·m (auto)','#aebed0')
        text(10,242,'Torque turns the body; it is NOT thrust.')
        text(10,270,'2  LOADED FEET PUSH AGAINST FLOOR')
        loaded=sum(n>0 for n in s.normal_loads)
        text(10,290,f'{loaded} loaded feet · normal load {sum(s.normal_loads):.2f} N')
        center=QPointF(72,355);radius=43.
        p.setPen(QPen(QColor('#38516d'),1))
        p.drawLine(center+QPointF(-radius,0),center+QPointF(radius,0))
        p.drawLine(center+QPointF(0,-radius),center+QPointF(0,radius))
        text(113,369,'+X');text(64,307,'+Y')
        fx,fy=s.force_xy;norm=math.hypot(fx,fy)
        length=min(radius,norm*8)
        if norm>1e-9:
            delta=QPointF(fx/norm*length,-fy/norm*length);tip=center+delta
            p.setPen(QPen(QColor('#ffd166'),3));p.drawLine(center,tip)
            unit=delta/max(length,1e-9);normal=QPointF(-unit.y(),unit.x())
            p.setBrush(QColor('#ffd166'))
            p.drawPolygon(QPolygonF([tip,tip-unit*7+normal*4,tip-unit*7-normal*4]))
        else:
            text(34,383,'no XY push','#ffd166')
        text(137,321,'WORLD XY FLOOR FORCE','#ffd166')
        text(137,341,f'Fx {fx:+.4f} N')
        text(137,361,f'Fy {fy:+.4f} N')
        text(137,381,'8 px/N · capped' if norm*8>radius else '8 px/N', '#aebed0')
        text(10,411,'3  VELOCITY CHANGE: Δv = F / m × Δt')
        text(10,431,f'Δvx {s.delta_v_xy[0]:+.6f} m/s')
        text(10,450,f'Δvy {s.delta_v_xy[1]:+.6f} m/s · mass {s.mass:.3f} kg')
        p.end()
