"""Vector and velocity explanation for an :class:`ImpulseSnapshot`.

The horizontal basis used here is the snapshot's fixed initial body -X basis;
it deliberately never follows the robot's later heading.
"""
from __future__ import annotations

import math
from typing import Iterable, Sequence

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QWidget


def project_horizontal(vector: Sequence[float], reference_xy: Sequence[float]) -> tuple[float, float]:
    """Return along/side projections in the fixed horizontal reference basis."""
    rx, ry = float(reference_xy[0]), float(reference_xy[1])
    norm = math.hypot(rx, ry)
    if norm <= 1e-15:
        raise ValueError("reference_xy must have a horizontal direction")
    rx, ry = rx / norm, ry / norm
    x, y = float(vector[0]), float(vector[1])
    return x * rx + y * ry, x * -ry + y * rx


def impulse_components(snapshot) -> tuple[float, float]:
    """Signed net impulse components (along, side), in N*s."""
    return project_horizontal(snapshot.net_impulse_ns, snapshot.reference_xy)


def initial_velocity_components(snapshot) -> tuple[float, float]:
    """Signed initial horizontal velocity components, in m/s."""
    return project_horizontal(snapshot.initial_velocity_mps, snapshot.reference_xy)


def velocity_components(snapshot) -> tuple[float, float]:
    """Compute ``initial velocity + net impulse / mass`` by component."""
    va0, vs0 = initial_velocity_components(snapshot)
    ja, js = impulse_components(snapshot)
    return va0 + ja / snapshot.mass_kg, vs0 + js / snapshot.mass_kg


def velocity_trace(snapshot) -> tuple[tuple[float, float, float], ...]:
    """Infer sampled (time, along velocity, side velocity) from cumulative J/m."""
    va0, vs0 = initial_velocity_components(snapshot)
    mass = snapshot.mass_kg
    return tuple((float(s[0]), va0 + float(s[1]) / mass, vs0 + float(s[2]) / mass)
                 for s in snapshot.samples)


def component_vector_sum(along: float, side: float) -> tuple[float, float]:
    """The net endpoint formed by head-to-tail along and side components."""
    return float(along), float(side)


class ImpulseVectorPanel(QWidget):
    """Draw impulse vectors, their velocity consequence, and sampled velocities."""
    def __init__(self):
        super().__init__()
        self.data = None
        self.setMinimumSize(380, 900)
        self.setAccessibleName("Impulse and velocity vectors in the fixed initial travel basis")

    def set_snapshot(self, data) -> None:
        self.data = data
        if data is None:
            self.setAccessibleDescription("No impulse samples yet")
        else:
            ja, js = impulse_components(data)
            self.setAccessibleDescription(
                f"Along impulse {ja:+.4f} newton seconds; side impulse {js:+.4f} "
                f"newton seconds; measured along displacement {data.along_displacement_m:+.4f} metres"
            )
        self.update()

    @staticmethod
    def _arrow(p: QPainter, start: QPointF, end: QPointF, color: str,
               width: int = 3, dashed: bool = False) -> None:
        style = Qt.PenStyle.DashLine if dashed else Qt.PenStyle.SolidLine
        p.setPen(QPen(QColor(color), width, style))
        p.drawLine(start, end)
        dx, dy = end.x() - start.x(), end.y() - start.y()
        length = math.hypot(dx, dy)
        if length < 1.0:
            p.setBrush(QColor(color)); p.drawEllipse(end, 2.5, 2.5)
            return
        ux, uy = dx / length, dy / length
        px, py = -uy, ux
        size = 8.0
        head = QPolygonF([end,
                          QPointF(end.x()-ux*size+px*size*.45, end.y()-uy*size+py*size*.45),
                          QPointF(end.x()-ux*size-px*size*.45, end.y()-uy*size-py*size*.45)])
        p.setPen(Qt.PenStyle.NoPen); p.setBrush(QColor(color)); p.drawPolygon(head)

    def paintEvent(self, event) -> None:
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#101b28"))
        margin, width = 12, min(380, self.width()-24)

        def text(rect, value, color="#d6e0ec", flags=Qt.AlignmentFlag.AlignLeft):
            p.setPen(QColor(color)); p.drawText(rect, int(flags | Qt.TextFlag.TextWordWrap), value)

        text(QRectF(margin, 8, width, 24), "IMPULSE VECTORS → VELOCITY → DISPLACEMENT", "#76d5f7")
        d = self.data
        if d is None:
            text(QRectF(margin, 38, width, 45), "Start or STEP to draw the fixed-basis vectors.")
            p.end(); return

        ja, js = impulse_components(d)
        side_color = "#52d6d8" if js >= 0 else "#ed70d4"
        maximum = max(abs(ja), abs(js), 1e-12)
        scale = 88.0 / maximum
        origin = QPointF(margin + width/2, 205)
        along_end = QPointF(origin.x()+ja*scale, origin.y())
        side_end = QPointF(origin.x(), origin.y()-js*scale)
        net_end = QPointF(along_end.x(), side_end.y())

        text(QRectF(margin, 34, width, 42),
             f"Fixed initial −X basis ({d.reference_xy[0]:+.2f}, {d.reference_xy[1]:+.2f}); not current heading.",
             "#aebed0")
        text(QRectF(margin, 78, width, 24),
             f"J along {ja:+.4f} N·s    J side {js:+.4f} N·s")
        p.setPen(QPen(QColor("#38516d"), 1))
        p.drawLine(QPointF(margin+8, origin.y()), QPointF(margin+width-8, origin.y()))
        p.drawLine(QPointF(origin.x(), 110), QPointF(origin.x(), 300))
        text(QRectF(margin+width-92, origin.y()+5, 90, 18), "+along →", "#ffd166",
             Qt.AlignmentFlag.AlignRight)
        text(QRectF(origin.x()+25, 95, 78, 18), "+side ↑", "#52d6d8")
        self._arrow(p, origin, along_end, "#ffd166", 4)
        self._arrow(p, origin, side_end, side_color, 4)
        self._arrow(p, side_end, net_end, "#ffd166", 2, True)
        self._arrow(p, along_end, net_end, side_color, 2, True)
        self._arrow(p, origin, net_end, "#ffffff", 3)
        text(QRectF(margin, 304, width, 22), f"Common impulse scale: 88 px = {maximum:.4g} N·s", "#aebed0")
        text(QRectF(margin, 326, width, 25), "Gold = along · cyan/magenta = signed side · white = net", "#aebed0")

        va0, vs0 = initial_velocity_components(d)
        dva, dvs = ja/d.mass_kg, js/d.mass_kg
        va, vs = va0+dva, vs0+dvs
        text(QRectF(margin, 365, width, 22), "VELOCITY DECOMPOSITION  [m/s]", "#76d5f7")
        text(QRectF(margin, 389, width, 42),
             f"Along: {va0:+.4f} + {dva:+.4f} = {va:+.4f}\nSide:   {vs0:+.4f} + {dvs:+.4f} = {vs:+.4f}")
        text(QRectF(margin, 433, width, 25),
             "initial velocity + J/m = current velocity", "#aebed0")
        # A separate velocity-scale vector addition: grey v0 followed head-to-tail
        # by green Δv, with the resulting current velocity in white.
        vmaximum = max(math.hypot(va0, vs0), math.hypot(dva, dvs),
                       math.hypot(va, vs), 1e-12)
        vscale = 48.0 / vmaximum
        vorigin = QPointF(margin+width/2, 535)
        v0end = vorigin + QPointF(va0*vscale, -vs0*vscale)
        vend = vorigin + QPointF(va*vscale, -vs*vscale)
        self._arrow(p, vorigin, v0end, "#aebed0", 2)
        self._arrow(p, v0end, vend, "#80ed99", 2, True)
        self._arrow(p, vorigin, vend, "#ffffff", 3)
        text(QRectF(margin, 463, width, 20),
             f"Velocity scale: 48 px = {vmaximum:.4g} m/s", "#aebed0")
        text(QRectF(margin, 590, width, 20), "grey v₀ + dashed green J/m = white current v", "#aebed0")

        trace = velocity_trace(d)
        text(QRectF(margin, 625, width, 22), "SIGNED VELOCITY OVER RETAINED TIME", "#76d5f7")
        text(QRectF(margin, 647, width, 20), "Gold = along · cyan = side", "#aebed0")
        plot = QRectF(margin+16, 670, width-32, 82)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(QColor("#38516d"), 1)); p.drawRect(plot)
        if trace:
            values = [v for row in trace for v in row[1:]] + [va0, vs0]
            vmax = max(max(abs(v) for v in values), 1e-9)
            t0, t1 = trace[0][0], trace[-1][0]
            p.drawLine(QPointF(plot.left(), plot.center().y()), QPointF(plot.right(), plot.center().y()))
            for column, color in ((1, "#ffd166"), (2, "#52d6d8")):
                points = [QPointF(plot.left()+(row[0]-t0)/max(t1-t0, 1e-12)*plot.width(),
                                  plot.center().y()-row[column]/vmax*(plot.height()/2-5)) for row in trace]
                p.setPen(QPen(QColor(color), 2)); p.drawPolyline(QPolygonF(points))
            text(QRectF(margin, 754, width, 20), f"Scale ±{vmax:.4g} m/s · {t0:.3f} to {t1:.3f} s", "#aebed0")
        else:
            text(plot, "No retained velocity samples yet", "#aebed0", Qt.AlignmentFlag.AlignCenter)

        text(QRectF(margin, 786, width, 86),
             f"A zero final impulse can still leave initial velocity and nonzero distance. "
             f"Distance is Σ v Δt, not impulse. Exact measured displacement: "
             f"along {d.along_displacement_m:+.4f} m, side {d.lateral_displacement_m:+.4f} m. "
             "The sampled trace is explanatory, not an exact area calculation.", "#d6e0ec")
        p.end()
