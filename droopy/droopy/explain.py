"""Auditable, plain-language derivations from one immutable consumed tick."""
from __future__ import annotations

from dataclasses import dataclass
import html
import math
import numpy as np


@dataclass(frozen=True)
class Check:
    """Named numerical comparison rendered as an unambiguous badge."""
    name: str
    expected: tuple[float, ...]
    actual: tuple[float, ...]
    atol: float = 1e-9
    rtol: float = 1e-7

    @property
    def ok(self) -> bool:
        return bool(np.allclose(self.expected, self.actual, atol=self.atol, rtol=self.rtol))

    def render(self, unit: str = "") -> str:
        badge = "✓ PASS" if self.ok else "✗ FAIL"
        color = "#80ed99" if self.ok else "#ff718d"
        residual = np.asarray(self.actual) - np.asarray(self.expected)
        return (f'<p><b style="color:{color}">{badge} — {html.escape(self.name)}</b><br>'
                f"expected {_v(self.expected, unit)}; recorded {_v(self.actual, unit)};<br>"
                f"residual {_v(residual, unit)} (atol {self.atol:g}, rtol {self.rtol:g})</p>")


@dataclass(frozen=True)
class Derivation:
    title: str
    body: str
    checks: tuple[tuple[Check, str], ...] = ()

    @property
    def ok(self) -> bool:
        return all(check.ok for check, _unit in self.checks)

    def render(self, common: str) -> str:
        if not self.checks:
            return common + self.body
        state = "✓ ALL CHECKS PASS" if self.ok else "✗ CHECKS FAILED"
        color = "#80ed99" if self.ok else "#ff718d"
        checks = "".join(check.render(unit) for check, unit in self.checks)
        return common + f'<h4 style="color:{color}">{state}</h4>' + self.body + checks


def _v(values, unit=""):
    return "(" + ", ".join(f"{x:+.5g}" for x in values) + f") {unit}"


def _motor_expected(d, config):
    """Reproduce clip and the final outward-at-speed gate in physics._single_step."""
    servo = config.servo_gain * (np.asarray(d.requested_rad_s)-np.asarray(d.rotor_speeds_before_rad_s))
    clipped = np.clip(servo, -config.max_torque, config.max_torque)
    speeds = np.asarray(d.rotor_speeds_before_rad_s)
    gated = np.where((speeds >= config.max_speed) & (clipped > 0), 0.0, clipped)
    return np.where((speeds <= -config.max_speed) & (gated < 0), 0.0, gated), servo, clipped


def derive(key, d, config, command=None, demand_time=None) -> Derivation:
    """Build prose, substitutions, and independently recomputed named checks."""
    if key == "control":
        if command is None:
            return Derivation("Control", "<p>Control demand unavailable.</p>")
        t = float(d.time_before_s if demand_time is None else demand_time)
        if not command.enabled:
            raw = (0.0, 0.0)
            formula = "Motors disabled → raw targets = (0, 0) rad/s (active braking)."
        elif command.mode == "Manual rotors":
            raw = (command.left, command.right)
            formula = (f"Manual: raw L={command.left:+.5g} rad/s; "
                       f"raw R={command.right:+.5g} rad/s.")
        else:
            if command.mode == "Constant counterspin":
                wave = 0.0
                wave_text = "Constant counterspin: wave = 0 rad/s."
            else:
                phase = 2*math.pi*command.frequency*t
                wave = command.amplitude*math.sin(phase)
                wave_text = (f"wave=A·sin(2πft₀)={command.amplitude:g}·sin(2π·"
                             f"{command.frequency:g}·{t:.6f})={wave:+.5g} rad/s.")
            raw = (command.spin+wave+command.bias, -command.spin+wave+command.bias)
            formula = (f"{wave_text}<br>L={command.spin:g}+({wave:+.5g})+({command.bias:+g})="
                       f"{raw[0]:+.5g}; R=−{command.spin:g}+({wave:+.5g})+"
                       f"({command.bias:+g})={raw[1]:+.5g} rad/s.")
        clipped = tuple(np.clip(raw, -config.max_speed, config.max_speed))
        body = ("<p><b>Why:</b> the open-loop gait creates rotor setpoints, not chassis travel.</p>"
                f"<p><b>Inputs and provenance:</b> mode/enabled and gait values are the command "
                f"consumed at t₀={t:.6f} s; ωmax={config.max_speed:g} rad/s comes from Settings.</p>"
                f"<p>{formula}<br>clip(raw, ±{config.max_speed:g})={_v(clipped,'rad/s')}.</p>"
                f"<p><b>Accepted/consumed tick:</b> raw targets {_v(d.targets_rad_s,'rad/s')}; "
                f"<b>requested after clamp:</b> {_v(d.requested_rad_s,'rad/s')}.</p>")
        return Derivation("Control", body, (
            (Check("gait raw targets", tuple(raw), tuple(d.targets_rad_s)), "rad/s"),
            (Check("speed clamp", clipped, tuple(d.requested_rad_s)), "rad/s"),
        ))

    if key == "motor":
        expected, servo, clipped = _motor_expected(d, config)
        speeds = np.asarray(d.rotor_speeds_before_rad_s)
        errors = np.asarray(d.requested_rad_s)-speeds
        gated = ["YES" if abs(expected[i]-clipped[i]) > 0 else "no" for i in range(2)]
        body = ("<p><b>Why:</b> each servo multiplies pre-step speed error by kₛ, clips to the "
                "torque limit, then suppresses only torque that pushes farther beyond ωmax.</p>"
                f"<p><b>Inputs:</b> requested {_v(d.requested_rad_s,'rad/s')} (speed clamp); "
                f"pre-step speed {_v(speeds,'rad/s')} (rotor state); kₛ={config.servo_gain:g} "
                f"N·m/(rad/s), τmax={config.max_torque:g} N·m, ωmax={config.max_speed:g} rad/s (Settings).</p>"
                f"<p>error=requested−speed={_v(errors,'rad/s')}<br>"
                f"servo=kₛ·error={config.servo_gain:g}·{_v(errors)}={_v(servo,'N·m')}<br>"
                f"torque clip={_v(clipped,'N·m')}<br>speed-bound outward gate L/R={gated[0]}/{gated[1]} "
                f"→ {_v(expected,'N·m')}.</p>")
        return Derivation("Motor torque", body, (
            (Check("motor torque clip + speed gate", tuple(expected), tuple(d.motor_torques_nm)), "N·m"),
        ))

    if key == "rotation":
        contact=np.asarray(d.contact_torque_body_nm)
        inertia=np.array([config.body_inertia_x,config.body_inertia_y,config.body_inertia_z])
        tilt=math.radians(config.tilt_deg)
        axes=np.array([[0.,math.sin(tilt),math.cos(tilt)],[0.,-math.sin(tilt),math.cos(tilt)]])
        omega=np.asarray(d.omega_before_rad_s)
        momentum=inertia*omega + axes.T @ (config.rotor_inertia*(axes@omega+np.asarray(d.rotor_speeds_before_rad_s)))
        reaction=-axes.T@np.asarray(d.motor_torques_nm)
        gyro=-np.cross(omega,momentum)
        expected=(contact+reaction+gyro)/inertia
        body=("<p><b>Why:</b> floor torque, motor reaction, and Euler gyroscopic torque turn the chassis.</p>"
              f"<p>J=diag{_v(inertia,'kg·m²')}; τcontact={_v(contact,'N·m')}; "
              f"−Aᵀτmotor={_v(reaction,'N·m')}; −ω×H={_v(gyro,'N·m')}.</p>"
              f"<p>α=(τcontact+τreaction+τgyro)/J={_v(expected,'rad/s²')}.</p>")
        body += f'<p>H = Jω + Iᵣ Aᵀ(Aω+s) = {_v(momentum,"kg·m²/s")}; axes reconstructed from tilt {config.tilt_deg:g}°.</p>'
        return Derivation("Reaction + gyro",body,(
            (Check('angular momentum',tuple(momentum),tuple(d.angular_momentum_body)),'kg·m²/s'),
            (Check('motor reaction',tuple(reaction),tuple(d.motor_reaction_nm)),'N·m'),
            (Check('gyroscopic torque',tuple(gyro),tuple(d.gyroscopic_nm)),'N·m'),
            (Check("Euler angular acceleration",tuple(expected),tuple(d.angular_acceleration_rad_s2)),"rad/s²")))

    if key == "contacts":
        rows=[]; checks=[]
        for c in d.contacts:
            pz,vz=c.point_world[2],c.velocity_world[2]
            normal=max(0.,-config.contact_stiffness*pz-config.contact_damping*vz) if pz < 0 else 0.
            vt=np.asarray(c.velocity_world[:2]); speed=float(np.linalg.norm(vt))
            friction=(-config.friction*normal*vt/math.sqrt(speed*speed+config.friction_velocity**2)
                      if normal > 0 else np.zeros(2))
            rows.append(f"<p><b>Foot {c.foot+1}</b><br>z={pz:+.5g} m; vz={vz:+.5g} m/s.<br>N = max(0, −{config.contact_stiffness:g} · ({pz:+.5g}) − {config.contact_damping:g} · ({vz:+.5g})) = {normal:.5g} N.<br>Friction = {_v(friction,'N')}</p>")
            checks.extend(((Check(f"foot {c.foot+1} normal",(normal,),(c.normal_n,)),"N"),
                           (Check(f"foot {c.foot+1} friction",tuple(friction),tuple(c.friction_n)),"N")))
        body=("<p>N=max(0,−k·z−c·vz); Fxy=−μN·vxy/√(|vxy|²+v₀²). Inputs are each pre-step foot position/velocity and Settings.</p>"
              +"".join(rows))
        return Derivation("Contacts + friction",body,tuple(checks))

    if key == "translation":
        force=np.sum([c.force_world_n for c in d.contacts],axis=0) if d.contacts else np.zeros(3)
        acceleration=force/config.mass+np.array([0.,0.,-config.gravity])
        velocity=np.asarray(d.velocity_before_mps)+d.dt_s*acceleration
        position=np.asarray(d.position_before_m)+d.dt_s*velocity
        body=(f"<p>a=ΣF/m+(0,0,−g)={_v(force,'N')}/{config.mass:g} kg+(0,0,−{config.gravity:g})="
              f"{_v(acceleration,'m/s²')}.</p><p>v₁={_v(d.velocity_before_mps,'m/s')}+{d.dt_s:g}·a={_v(velocity,'m/s')}; "
              f"x₁={_v(d.position_before_m,'m')}+{d.dt_s:g}·v₁={_v(position,'m')}.</p><p>Recompute residuals are audited below.</p>")
        return Derivation("Translation",body,(
            (Check('sum of foot forces',tuple(force),tuple(d.contact_force_n)),'N'),
            (Check("force to acceleration",tuple(acceleration),tuple(d.linear_acceleration_mps2)),"m/s²"),
            (Check("velocity integration",tuple(velocity),tuple(d.velocity_after_mps)),"m/s"),
            (Check("position integration",tuple(position),tuple(d.position_after_m)),"m")))

    if key == "energy":
        delta=d.energy_after_j-d.energy_before_j
        return Derivation("Energy audit",("<p>Mechanical energy includes translation, gravity, chassis rotation, rotor axial motion, and contact springs.</p>"
            f"<p>E₀={d.energy_before_j:.8g} J; E₁={d.energy_after_j:.8g} J; ΔE={delta:+.6g} J.<br>"
            f"Midpoint motor work={d.motor_work_j:+.6g} J; ΔE−Wmotor={delta-d.motor_work_j:+.6g} J "
            "(contact losses plus integration residual).</p>"))
    return Derivation(key, "")


def explanation(key, diagnostic, config, command=None, demand_time=None) -> str:
    if diagnostic is None:
        return "<h3>No completed tick</h3><p>Step or start the simulation. Reset clears old evidence so values cannot be mistaken for the new run.</p>"
    d=diagnostic
    common=(f"<h3>{html.escape(key.replace('_',' ').title())}</h3><p><b>Consumed tick:</b> "
            f"{d.time_before_s:.6f} → {d.time_before_s+d.dt_s:.6f} s (Δt={d.dt_s:.6g} s).</p>")
    return derive(key,d,config,command,demand_time).render(common)


KEYS = (("control", "Setpoint + speed control"), ("motor", "Motor torque"), ("rotation", "Reaction + gyro"),
        ("contacts", "Contacts + friction"), ("translation", "Translation"), ("energy", "Energy audit"))
