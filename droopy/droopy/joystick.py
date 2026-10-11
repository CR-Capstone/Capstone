"""Melty-style world direction demand with separately measured travel."""
import math
from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QWidget


class DirectionJoystick(QWidget):
    changed = Signal(float,float)

    def __init__(self):
        super().__init__()
        self.angle_deg=180.; self.strength=0.; self.velocity=(0.,0.)
        self.setMinimumSize(200,200)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setAccessibleName('Requested world travel direction')
        self.setToolTip('Drag yellow demand: direction and strength. Cyan arrow is actual world velocity. Right-click, double-click or Space to centre. This experimental controller cannot guarantee tracking.')

    def set_value(self, angle, strength):
        self.angle_deg=angle; self.strength=max(0.,min(1.,strength)); self.update()

    def set_velocity(self,vx,vy):
        self.velocity=(vx,vy); self.update()

    def geometry_values(self):
        return QPointF(self.width()/2,self.height()/2),min(self.width(),self.height())/2-28

    def move_to(self,pos):
        c,r=self.geometry_values(); x=pos.x()-c.x(); y=c.y()-pos.y()
        self.set_value(math.degrees(math.atan2(y,x)),math.hypot(x,y)/max(1,r))
        self.changed.emit(self.angle_deg,self.strength)

    def centre(self):
        self.set_value(self.angle_deg,0.);self.changed.emit(self.angle_deg,0.)

    def mousePressEvent(self,e):
        if e.button()==Qt.MouseButton.RightButton:self.centre()
        elif e.button()==Qt.MouseButton.LeftButton:self.move_to(e.position())

    def mouseMoveEvent(self,e):
        if e.buttons() & Qt.MouseButton.LeftButton:self.move_to(e.position())

    def mouseDoubleClickEvent(self,e):self.centre()

    def keyPressEvent(self,e):
        directions={Qt.Key.Key_Up:90,Qt.Key.Key_Down:-90,Qt.Key.Key_Left:180,Qt.Key.Key_Right:0}
        if e.key()==Qt.Key.Key_Space:self.centre()
        elif e.key() in directions:
            self.set_value(directions[e.key()],max(.5,self.strength)); self.changed.emit(self.angle_deg,self.strength)
        else:super().keyPressEvent(e)

    def paintEvent(self,e):
        p=QPainter(self);p.setRenderHint(QPainter.RenderHint.Antialiasing)
        c,r=self.geometry_values();p.fillRect(self.rect(),QColor('#0e1520'))
        p.setBrush(Qt.BrushStyle.NoBrush)
        for fraction in (.25,.5,.75,1):
            p.setPen(QPen(QColor('#38516d' if fraction==1 else '#24354a'),1));p.drawEllipse(c,r*fraction,r*fraction)
        p.drawLine(QPointF(c.x()-r,c.y()),QPointF(c.x()+r,c.y()))
        p.drawLine(QPointF(c.x(),c.y()-r),QPointF(c.x(),c.y()+r))
        p.setPen(QColor('#aebed0'))
        for dx,dy,text in [(r+14,0,'+X'),(-r-14,0,'−X'),(0,-r-14,'+Y'),(0,r+14,'−Y')]:
            p.drawText(QRectF(c.x()+dx-15,c.y()+dy-9,30,18),Qt.AlignmentFlag.AlignCenter,text)
        angle=math.radians(self.angle_deg)
        tip=QPointF(c.x()+math.cos(angle)*r*self.strength,c.y()-math.sin(angle)*r*self.strength)
        p.setPen(QPen(QColor('#ffd166'),3));p.drawLine(c,tip);p.setBrush(QColor('#ffd166'));p.drawEllipse(tip,6,6)
        vx,vy=self.velocity;speed=math.hypot(vx,vy)
        if speed>.0001:
            tip=QPointF(c.x()+r*vx/speed,c.y()-r*vy/speed)
            p.setPen(QPen(QColor('#76d5f7'),2));p.drawLine(c,tip);p.setBrush(QColor('#76d5f7'));p.drawEllipse(tip,4,4)
        p.setPen(QColor('#d6e0ec'));p.drawText(8,16,f'Demand {self.angle_deg:+.0f}° · {self.strength:.0%}')
        p.end()
