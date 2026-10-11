"""Exact consumed-tick torque axes, transformed from body to world coordinates."""
from dataclasses import dataclass
import numpy as np
from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QColor, QPen, QPolygonF


@dataclass(frozen=True)
class TorqueVector:
    name: str
    color: str
    body_nm: tuple
    world_nm: tuple


def torque_vectors(diagnostic):
    if diagnostic is None:
        return ()
    rotation=np.asarray(diagnostic.rotation_before).reshape(3,3)
    motor=np.asarray(diagnostic.motor_reaction_nm)
    gyro=np.asarray(diagnostic.gyroscopic_nm)
    floor=np.asarray(diagnostic.contact_torque_body_nm)
    return tuple(TorqueVector(name,color,tuple(vector),tuple(rotation@vector))
                 for name,color,vector in (
                     ('motor reaction','#ff796d',motor),
                     ('Euler/gyro term','#b794f4',gyro),
                     ('floor torque','#ffd166',floor),
                     ('effective total','#f7f7ff',motor+gyro+floor)))


def draw_torques(painter,diagnostic,project_world,width):
    """Draw double-headed axial vectors; return inspector hit segments.

    One physical length/Nm scale is used before camera projection, so
    foreshortened arrows are NOT comparable by their screen lengths alone.
    """
    vectors=torque_vectors(diagnostic)
    if not vectors:
        painter.setPen(QColor('#aebed0'))
        painter.drawText(18,72,'Torque axes: STEP to generate evidence.')
        return []
    maximum=max(.01,max(np.linalg.norm(v.world_nm) for v in vectors))
    scale=.14/maximum
    origin=np.asarray(diagnostic.position_before_m)
    start=project_world(origin)
    hits=[]
    # Total underlaid so thin component arrows remain distinguishable.
    for v in reversed(vectors):
        norm=np.linalg.norm(v.world_nm)
        if norm<1e-12:continue
        end=project_world(origin+np.asarray(v.world_nm)*scale)
        delta=end-start;length=np.hypot(delta.x(),delta.y())
        color=QColor(v.color)
        painter.setPen(QPen(color,4 if v.name=='effective total' else 2))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        if length<3:
            # Nonzero axis viewed end-on: never silently draw it as zero.
            painter.drawEllipse(start,6,6)
        else:
            painter.drawLine(start,end)
            unit=delta/length;normal=QPointF(-unit.y(),unit.x())
            painter.setBrush(color)
            for back in (0.,7.):
                tip=end-unit*back
                painter.drawPolygon(QPolygonF([tip,tip-unit*6+normal*3,tip-unit*6-normal*3]))
        hits.append(('rotation',start,end))
    painter.setPen(QColor('#d6e0ec'))
    painter.drawText(18,72,f'TORQUE AXES [N·m] · right-hand rule · tick {diagnostic.time_before_s:.4f} s')
    for i,v in enumerate(vectors):
        # Two columns; legend carries magnitude even for end-on axes.
        x=18+(i%2)*max(220,(width-36)/2);y=93+(i//2)*20
        painter.setPen(QColor(v.color))
        painter.drawText(int(x),y,f'{v.name}: {np.linalg.norm(v.world_nm):.3f} N·m')
    painter.setPen(QColor('#aebed0'))
    painter.drawText(18,134,f'Auto scale {scale:.4f} m/(N·m), before projection · circle = end-on')
    painter.drawText(18,154,'Double heads = rotation axis, NOT thrust. Gyro = Euler coupling.')
    return hits
