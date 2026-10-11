"""Top-down world-plane projection of recorded floor reaction forces."""
import math
from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QWidget


class ContactSchematic(QWidget):
    inspect_requested=Signal(str)

    def __init__(self,parent=None):
        super().__init__(parent)
        self.contacts=()
        self.stamp='No completed step'
        self.setMinimumHeight(470)
        self.setMinimumWidth(320)
        self.setAccessibleName('Loaded feet and projected horizontal floor forces')
        self.setToolTip('Yellow dots carry vertical support force; gray dots are unloaded. Dashed vectors relocate each horizontal force to a common origin without changing its direction or size. Cyan is their vector sum. Click for contact math.')

    @property
    def projected_forces(self):
        # Diagnostics already use world coordinates. Orthogonal projection onto
        # the floor drops z; this is not a rotation, prediction, or new force.
        return tuple(tuple(c.force_world_n[:2]) for c in self.contacts)

    @property
    def net_force_xy(self):
        return tuple(sum(f[i] for f in self.projected_forces) for i in (0,1))

    def set_contacts(self,contacts,stamp=None):
        self.contacts=tuple(contacts or ())
        if stamp is not None:self.stamp=stamp
        elif not self.contacts:self.stamp='No completed step'
        loaded=sum(c.normal_n>0 for c in self.contacts)
        self.setAccessibleDescription(f'{self.stamp}; yellow means loaded feet ({loaded}); gray means unloaded. Horizontal projection Fx,Fy={self.net_force_xy} N; vertical Fz is support, excluded from XY ghost vectors.')
        self.update()

    def mousePressEvent(self,event):
        if event.button()==Qt.MouseButton.LeftButton:self.inspect_requested.emit('contacts')

    def paintEvent(self,event):
        p=QPainter(self);p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(),QColor('#0e1722'))
        def text(x,y,s,color='#d6e0ec'):
            p.setPen(QColor(color));p.drawText(x,y,s)
        def arrow(origin,force,scale,color,dashed=False,width=2):
            fx,fy=force
            if math.hypot(fx,fy)<1e-10:return
            delta=QPointF(fx*scale,-fy*scale);tip=origin+delta
            pen=QPen(QColor(color),width,Qt.PenStyle.DashLine if dashed else Qt.PenStyle.SolidLine)
            p.setPen(pen);p.drawLine(origin,tip)
            norm=math.hypot(delta.x(),delta.y());u=delta/max(norm,1e-12);n=QPointF(-u.y(),u.x())
            p.setBrush(QColor(color));p.drawPolygon(QPolygonF([tip,tip-u*6+n*3,tip-u*6-n*3]))
        text(10,20,'YELLOW DOTS = LOADED FEET','#ffd166')
        text(10,40,'Gray = unloaded · numbers identify each foot')
        text(10,60,self.stamp,'#aebed0')
        text(10,82,'FLOOR VIEW: +X right, +Y up; height removed')
        w=self.width();left=QPointF(w*.26,180);right=QPointF(w*.75,180)
        radius=min(53.,w*.15)
        text(10,107,'At the feet');text(int(w*.52),107,'Ghosts at one origin')
        for center in (left,right):
            p.setPen(QPen(QColor('#38516d'),1))
            p.drawLine(center-QPointF(radius,0),center+QPointF(radius,0))
            p.drawLine(center-QPointF(0,radius),center+QPointF(0,radius))
            text(int(center.x()+radius),int(center.y()+13),'+X','#7f91a8')
            text(int(center.x()-8),int(center.y()-radius-5),'+Y','#7f91a8')
        forces=self.projected_forces;net=self.net_force_xy
        largest=max([1.,math.hypot(*net)]+[math.hypot(*f) for f in forces])
        force_scale=radius/largest  # One common force scale; no per-arrow normalization.
        if self.contacts:
            x0=sum(c.point_world[0] for c in self.contacts)/len(self.contacts)
            y0=sum(c.point_world[1] for c in self.contacts)/len(self.contacts)
            extent=max(.001,max(math.hypot(c.point_world[0]-x0,c.point_world[1]-y0) for c in self.contacts))
            position_scale=radius*.42/extent
            colors=('#ffbd69','#c8a5ff','#90d7a1','#f48caf')
            points=[]
            for c,f in zip(self.contacts,forces):
                point=left+QPointF((c.point_world[0]-x0)*position_scale,-(c.point_world[1]-y0)*position_scale)
                points.append(point)
                color=colors[c.foot%4]
                arrow(point,f,force_scale,color)
                arrow(right,f,force_scale,color,True)
                p.setPen(QColor('#111925'));p.setBrush(QColor('#ffd166' if c.normal_n>0 else '#536578'))
                p.drawEllipse(point,8,8)
                text(int(point.x()-4),int(point.y()+4),str(c.foot+1),'#111925' if c.normal_n>0 else '#d6e0ec')
            arrow(right,net,force_scale,'#76d5f7',False,3)
        else:text(12,185,'Step or capture to show forces.','#aebed0')
        text(10,257,f'Force scale: {force_scale:.2f} px/N (shared, auto)','#aebed0')
        text(10,277,'Foot spacing schematic; force directions are world XY.','#aebed0')
        text(10,300,'Dashed = individual Fx,Fy; cyan = sum','#76d5f7')
        text(10,320,f'Net push: Fx {net[0]:+.3f} N · Fy {net[1]:+.3f} N')
        text(10,341,'Projection: (Fx, Fy, Fz) → (Fx, Fy, 0)')
        text(10,362,'Vertical Fz supports weight; it is not forward push.')
        for i,c in enumerate(self.contacts):
            f=self.projected_forces[i]
            text(10,385+i*19,f'Foot {c.foot+1}: N {c.normal_n:6.2f} N · XY ({f[0]:+.2f}, {f[1]:+.2f}) N', '#ffd166' if c.normal_n>0 else '#8999ac')
        if self.contacts and math.hypot(*net)<1e-9:text(10,466,'Net XY push is zero (individual forces may cancel).','#76d5f7')
        p.end()
