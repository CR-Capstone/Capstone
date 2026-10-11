"""Live, inspectable block diagram of one completed physics tick.

The view intentionally renders only values from a :class:`StepDiagnostic`.  It
never reads a running Simulator, so every number on screen has one timestamp.
"""
from __future__ import annotations

from copy import deepcopy
import math

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox, QFrame, QHBoxLayout, QLabel, QPushButton, QSizePolicy,
    QVBoxLayout, QWidget,
)


_KEYS = {"control", "motor", "rotation", "contacts", "translation", "energy"}


def _vector(values, unit: str = "") -> str:
    text = "(" + ", ".join(f"{float(value):+.5g}" for value in values) + ")"
    return f"{text} {unit}" if unit else text


def _matrix(values) -> str:
    rows = tuple(values[index:index + 3] for index in range(0, 9, 3))
    return "[" + "; ".join(", ".join(f"{float(x):+.3f}" for x in row) for row in rows) + "]"


class _Block(QFrame):
    """A wrapping, keyboard-readable diagram block with one inspection target."""

    activated = Signal(str)

    def __init__(self, key: str, title: str, parent=None):
        super().__init__(parent)
        self.key = key
        self.setObjectName(f"system_{key}_block")
        self.setFrameShape(QFrame.Shape.StyledPanel)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setToolTip(f"Click heading or Inspect for {title.lower()} derivation; body text is selectable")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(9, 7, 9, 7)
        layout.setSpacing(3)
        heading = QLabel(title)
        heading.setObjectName("systemBlockTitle")
        heading.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.body = QLabel()
        self.body.setWordWrap(True)
        self.body.setTextFormat(Qt.TextFormat.PlainText)
        self.body.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(heading)
        layout.addWidget(self.body)
        self.inspect_button = QPushButton('Inspect this calculation')
        self.inspect_button.clicked.connect(lambda: self.activated.emit(self.key))
        layout.addWidget(self.inspect_button)

    def setText(self, text: str) -> None:
        self.body.setText(text)

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self.activated.emit(self.key)
        super().mousePressEvent(event)


class SystemView(QWidget):
    """Vertical, live block diagram for the latest immutable consumed tick."""

    inspect_requested = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.latest_diagnostic = None
        self.latest_config = None
        self.diagnostic = None
        self.config = None
        self.frozen = False

        self.setMinimumWidth(340)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.MinimumExpanding)
        self.setAccessibleName("Live physics system block diagram")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(7, 7, 7, 9)
        layout.setSpacing(5)
        title = QLabel("LIVE SYSTEM — ONE CONSUMED PHYSICS TICK")
        title.setObjectName("section")
        title.setWordWrap(True)
        layout.addWidget(title)

        controls = QHBoxLayout()
        self.freeze = QCheckBox("Freeze")
        self.freeze.setToolTip("Hold one complete tick and its settings snapshot")
        self.freeze.toggled.connect(self.set_frozen)
        self.capture = QPushButton("Capture latest")
        self.capture.clicked.connect(self.capture_latest)
        controls.addWidget(self.freeze)
        controls.addWidget(self.capture)
        controls.addStretch(1)
        layout.addLayout(controls)

        self.state_label = QLabel("Live · no completed tick")
        self.state_label.setWordWrap(True)
        layout.addWidget(self.state_label)

        self.blocks: dict[str, _Block] = {}
        specs = (
            ("control", "1 · CONSUMED COMMAND / CLAMP"),
            ("motor", "2 · BOUNDED MOTOR SERVO"),
            ("rotation", "3 · BODY ROTATION (BODY FRAME)"),
            ("contacts", "4 · FLOOR CONTACT (WORLD FORCE)"),
            ("translation", "5 · WORLD TRANSLATION"),
            ("energy", "6 · STATE FEEDBACK — NEXT TICK"),
        )
        for index, (key, heading) in enumerate(specs):
            block = _Block(key, heading)
            block.activated.connect(self.inspect_requested)
            self.blocks[key] = block
            layout.addWidget(block)
            if index != len(specs) - 1:
                arrow = QLabel((
                    '↓ clamped setpoints to servo',
                    '↓ motor reaction enters rotation',
                    '↥ floor torque feeds rotation above\n↳ contacts evaluated from shared PRE-STEP state',
                    '↓ world floor force feeds translation',
                    '↓ integrated state feeds NEXT tick',
                )[index])
                arrow.setWordWrap(True)
                arrow.setAlignment(Qt.AlignmentFlag.AlignCenter)
                arrow.setObjectName("systemArrow")
                layout.addWidget(arrow)
        layout.addStretch(1)

        self.setStyleSheet("""
            _Block { background: #132131; border: 1px solid #38516d; border-radius: 5px; }
            _Block:hover { border-color: #76d5f7; }
            QLabel#systemBlockTitle { color: #76d5f7; font-weight: bold; }
            QLabel#systemArrow { color: #ffd166; font-size: 18px; font-weight: bold; }
        """)
        self._show(None, None)

    def update_tick(self, diagnostic, config) -> None:
        """Accept the newest completed tick and its matching settings.

        Both references are snapshotted together.  In particular, mutable Config
        edits made after this call cannot alter a frozen diagram.
        """
        if diagnostic is None:
            self.latest_diagnostic = None
            self.latest_config = None
        else:
            if config is None:
                raise ValueError("config is required for a completed tick")
            self.latest_diagnostic = deepcopy(diagnostic)
            self.latest_config = deepcopy(config)
        if not self.frozen:
            self._show(self.latest_diagnostic, self.latest_config)

    def set_frozen(self, enabled: bool) -> None:
        self.frozen = bool(enabled)
        if not self.frozen:
            self._show(self.latest_diagnostic, self.latest_config)
        else:
            # _show already owns copies, but copy once more at the freeze boundary
            # to make that ownership explicit and robust to non-frozen callers.
            self.diagnostic = deepcopy(self.diagnostic)
            self.config = deepcopy(self.config)
            self._render()

    def capture_latest(self) -> None:
        """Freeze and replace the display with the newest complete snapshot."""
        self.freeze.setChecked(True)
        self._show(self.latest_diagnostic, self.latest_config)

    def reset(self) -> None:
        self.latest_diagnostic = None
        self.latest_config = None
        self.diagnostic = None
        self.config = None
        self.freeze.setChecked(False)
        self._show(None, None)

    def _show(self, diagnostic, config) -> None:
        self.diagnostic = deepcopy(diagnostic)
        self.config = deepcopy(config)
        self._render()

    def _render(self) -> None:
        d, c = self.diagnostic, self.config
        if d is None or c is None:
            mode = "Frozen" if self.frozen else "Live"
            self.state_label.setText(f"{mode} · no completed tick")
            self.setAccessibleDescription("No completed physics tick. Step or start the simulation.")
            messages = {
                "control": "No consumed command yet.",
                "motor": "No servo result yet.",
                "rotation": "No body torque result yet.",
                "contacts": "No pre-step foot contacts yet.",
                "translation": "No world-force integration yet.",
                "energy": "NEXT TICK feedback unavailable until a step completes.",
            }
            for key, text in messages.items():
                self.blocks[key].setText(text)
            return

        t1 = d.time_before_s + d.dt_s
        mode = "Frozen snapshot" if self.frozen else "Live latest"
        self.state_label.setText(
            f"{mode} · consumed {d.time_before_s:.6f} → {t1:.6f} s · Δt {d.dt_s:.6g} s"
        )

        errors = tuple(req - speed for req, speed in zip(
            d.requested_rad_s, d.rotor_speeds_before_rad_s
        ))
        servo = tuple(c.servo_gain * value for value in errors)
        self.blocks["control"].setText(
            f"raw target = {_vector(d.targets_rad_s, 'rad/s')}\n"
            f"clamp(raw, ±ωmax={c.max_speed:g}) = {_vector(d.requested_rad_s, 'rad/s')}\n"
            f"same-tick pre-step rotor speed s₀ = {_vector(d.rotor_speeds_before_rad_s, 'rad/s')}"
        )
        self.blocks["motor"].setText(
            f"error = requested − s₀ = {_vector(errors, 'rad/s')}\n"
            f"kₛ·error = {_vector(servo, 'N·m')}\n"
            f"τmotor = bounded/gated result = {_vector(d.motor_torques_nm, 'N·m')}\n"
            f"bounds: |τ| ≤ {c.max_torque:g} N·m; outward torque gated at |s| ≥ {c.max_speed:g} rad/s\n"
            "Motors exchange angular momentum; they provide NO direct thrust."
        )
        inertia = (c.body_inertia_x, c.body_inertia_y, c.body_inertia_z)
        self.blocks["rotation"].setText(
            f"H = Jb·ω + Iᵣ Aᵀ(Aω+s) = {_vector(d.angular_momentum_body, 'kg·m²/s')}\n"
            f"τreaction = −Σ aᵢuᵢ = {_vector(d.motor_reaction_nm, 'N·m')}\n"
            f"τgyro = −ω×H = {_vector(d.gyroscopic_nm, 'N·m')}\n"
            f"τfloor(body) = {_vector(d.contact_torque_body_nm, 'N·m')}\n"
            f"Jb={_vector(inertia, 'kg·m²')} · α = floor + reaction + gyro\n"
            f"α = {_vector(d.angular_acceleration_rad_s2, 'rad/s²')}\n"
            "Gyro is the Euler coupling term, NOT an independent external torque."
        )

        contact_lines = [
            "Contacts use SAME-TICK pre-step position, velocity, and attitude R₀.",
            f"If z<0: N=max(0, −kz−cż); otherwise N=0. k={c.contact_stiffness:g} N/m, c={c.contact_damping:g} N·s/m",
            f"friction: |Ft| ≤ μN, μ={c.friction:g}",
        ]
        for contact in d.contacts:
            ft = math.hypot(*contact.friction_n)
            contact_lines.append(
                f"foot {contact.foot + 1}: z={contact.point_world[2]:+.5g} m, "
                f"vz={contact.velocity_world[2]:+.5g} m/s, N={contact.normal_n:.5g} N, "
                f"Ft={_vector(contact.friction_n, 'N')}, |Ft|={ft:.5g} ≤ μN={c.friction * contact.normal_n:.5g} N"
            )
        contact_lines.append(f"ΣFfloor = {_vector(d.contact_force_n, 'N')} world")
        self.blocks["contacts"].setText("\n".join(contact_lines))

        weight = (0.0, 0.0, -c.mass * c.gravity)
        total_force = tuple(d.contact_force_n[i] + weight[i] for i in range(3))
        self.blocks["translation"].setText(
            f"Fworld = Ffloor + (0,0,−mg)\n"
            f"= {_vector(d.contact_force_n, 'N')} + {_vector(weight, 'N')} = {_vector(total_force, 'N')}\n"
            f"a = Fworld/m = {_vector(d.linear_acceleration_mps2, 'm/s²')}\n"
            f"v₁ = v₀ + aΔt: {_vector(d.velocity_before_mps, 'm/s')} → {_vector(d.velocity_after_mps, 'm/s')}\n"
            f"x₁ = x₀ + v₁Δt: {_vector(d.position_before_m, 'm')} → {_vector(d.position_after_m, 'm')}"
        )
        self.blocks["energy"].setText(
            f"ω₁ = ω₀ + αΔt: {_vector(d.omega_before_rad_s, 'rad/s')} → {_vector(d.omega_after_rad_s, 'rad/s')}\n"
            f"R₀ body→world = {_matrix(d.rotation_before)}\n"
            "attitude R₁ is integrated from midpoint ω and normalized.\n"
            "FEEDBACK BRANCH ↺ NEXT TICK: (x₁, v₁, ω₁, R₁) → foot geometry/velocity → contact.\n"
            "Important: this tick's contacts were evaluated at the pre-step state R₀, not the new attitude.\n"
            f"Energy audit: E₀={d.energy_before_j:.6g} J, E₁={d.energy_after_j:.6g} J, motor work={d.motor_work_j:+.6g} J."
        )
        self.setAccessibleDescription(
            f"{mode}, tick {d.time_before_s:.6f} to {t1:.6f} seconds. "
            "Body torque is in body coordinates; contact and translation are in world coordinates."
        )
