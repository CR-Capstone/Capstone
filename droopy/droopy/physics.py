"""Exploratory rigid-body model for a twin angled-rotor shuffle robot.

This is an honest *similar* mechanism, not a calibrated replica of Droopy: public
dimensions, mass properties, foot construction, and gait data for that robot are
not available.  Motors never apply a world force.  They exchange angular
momentum with a finite-inertia chassis; translation can arise only through
unilateral floor contacts.

Let ``a_i`` be a rotor axis fixed in the body, ``s_i`` its rate relative to the
body, and ``I_r`` its axial inertia.  The kinetic energy and body-frame angular
momentum used here are

    T = 1/2 w.J_b.w + sum 1/2 I_r (a_i.w + s_i)^2
    H = J_b w + sum I_r a_i (a_i.w + s_i).

For motor torque ``u_i`` on the relative rotor coordinate,
``I_r(a_i.alpha + sdot_i)=u_i``.  Combining this with Euler's equation gives

    J_b alpha = tau_external - sum(a_i u_i) - w x H.

This is equivalent to solving the coupled equations with locked inertia
``J_b + sum(I_r a_i a_i.T)`` and its rotor Schur complement; it avoids treating
motor reaction as an external torque.  A normalized quaternion advances the
body-to-world attitude.  Feet use penalty normal forces and regularized Coulomb
friction, always bounded by ``mu*N``.

Limitations include lumped diagonal inertia, axial-only rotor inertia, ideal
bounded torque/speed servos, a flat rigid plane, point feet, and no flex,
collision geometry, electrical, thermal, or aerodynamic model.  Explicit fixed
steps require contact stiffness/damping to be tuned with ``dt``; the defaults
are intended for exploratory visualization, not design or safety validation.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
import math
import numpy as np


def _tuple(values) -> tuple[float, ...]:
    """Make a diagnostic value independent of later mutable simulator state."""
    return tuple(float(value) for value in np.asarray(values).ravel())


@dataclass(frozen=True)
class ContactDiagnostic:
    foot: int
    point_world: tuple[float, ...]
    velocity_world: tuple[float, ...]
    penetration_m: float
    normal_n: float
    friction_n: tuple[float, ...]
    force_world_n: tuple[float, ...]


@dataclass(frozen=True)
class StepDiagnostic:
    """Immutable audit trail for the most recently *completed* physics tick.

    All values are copied to Python scalars/tuples.  Thus an explainer can safely
    recompute the tick even while the live Simulator advances or is reset.
    """
    time_before_s: float
    dt_s: float
    targets_rad_s: tuple[float, ...]
    requested_rad_s: tuple[float, ...]
    position_before_m: tuple[float, ...]
    velocity_before_mps: tuple[float, ...]
    omega_before_rad_s: tuple[float, ...]
    rotation_before: tuple[float, ...]
    rotor_speeds_before_rad_s: tuple[float, ...]
    contacts: tuple[ContactDiagnostic, ...]
    contact_force_n: tuple[float, ...]
    contact_torque_body_nm: tuple[float, ...]
    motor_torques_nm: tuple[float, ...]
    angular_momentum_body: tuple[float, ...]
    motor_reaction_nm: tuple[float, ...]
    gyroscopic_nm: tuple[float, ...]
    angular_acceleration_rad_s2: tuple[float, ...]
    rotor_acceleration_rad_s2: tuple[float, ...]
    linear_acceleration_mps2: tuple[float, ...]
    omega_after_rad_s: tuple[float, ...]
    rotor_speeds_after_rad_s: tuple[float, ...]
    velocity_after_mps: tuple[float, ...]
    position_after_m: tuple[float, ...]
    energy_before_j: float
    energy_after_j: float
    motor_work_j: float


@dataclass
class Config:
    dt: float = 0.0005
    mass: float = 1.36
    tilt_deg: float = 15.0
    friction: float = 0.7
    rotor_inertia: float = 0.00035
    max_torque: float = 0.8
    max_speed: float = 450.0
    gravity: float = 9.81

    # Approximate geometry and numerical contact parameters (SI units).
    body_inertia_x: float = 0.012
    body_inertia_y: float = 0.016
    body_inertia_z: float = 0.020
    rotor_offset_y: float = 0.105
    rotor_height: float = 0.0
    # Narrow experimental support permits load transfer and shuffling. These
    # are contact positions, not the footprint of the visual chassis.
    foot_half_length: float = 0.025
    foot_half_width: float = 0.025
    foot_drop: float = 0.055
    contact_stiffness: float = 18000.0
    contact_damping: float = 90.0
    friction_velocity: float = 0.015
    servo_gain: float = 0.025

    def validate(self) -> None:
        """Raise ``ValueError`` if any parameter is nonfinite or unphysical."""
        for item in fields(self):
            value = getattr(self, item.name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{item.name} must be a finite number")
            if not math.isfinite(float(value)):
                raise ValueError(f"{item.name} must be finite")

        positive = (
            "dt", "mass", "rotor_inertia", "max_torque", "max_speed",
            "body_inertia_x", "body_inertia_y", "body_inertia_z",
            "rotor_offset_y", "foot_half_length", "foot_half_width",
            "foot_drop", "contact_stiffness", "contact_damping",
            "friction_velocity", "servo_gain",
        )
        for name in positive:
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.gravity < 0.0:
            raise ValueError("gravity must be nonnegative")
        if self.friction < 0.0:
            raise ValueError("friction must be nonnegative")
        if not (0.0 <= self.tilt_deg < 90.0):
            raise ValueError("tilt_deg must be in [0, 90)")


class Simulator:
    """Deterministic fixed-step 3-D robot dynamics simulator."""

    def __init__(self, config: Config | None = None):
        self.config = config if config is not None else Config()
        self.config.validate()
        c = self.config
        tilt = math.radians(c.tilt_deg)
        sy, cz = math.sin(tilt), math.cos(tilt)
        # Left rotor is at +y and tilts toward +y; right mirrors it.
        self.axes_body = np.array([[0.0, sy, cz], [0.0, -sy, cz]], dtype=float)
        self.rotor_centers_body = np.array(
            [[0.0, c.rotor_offset_y, c.rotor_height],
             [0.0, -c.rotor_offset_y, c.rotor_height]], dtype=float,
        )
        self.feet_body = np.array(
            [[x, y, -c.foot_drop]
             for x in (-c.foot_half_length, c.foot_half_length)
             for y in (-c.foot_half_width, c.foot_half_width)], dtype=float,
        )
        self._J_body = np.diag(
            [c.body_inertia_x, c.body_inertia_y, c.body_inertia_z]
        )
        self._J_body_inv = np.diag(1.0 / np.diag(self._J_body))
        self.reset()

    def reset(self) -> None:
        self.time = 0.0
        self.position = np.array([0.0, 0.0, self.config.foot_drop], dtype=float)
        self.velocity = np.zeros(3, dtype=float)
        self.omega = np.zeros(3, dtype=float)
        self.rotation = np.eye(3, dtype=float)
        self._quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.rotor_speeds = np.zeros(2, dtype=float)
        self.rotor_angles = np.zeros(2, dtype=float)
        self.last_torques = np.zeros(2, dtype=float)
        self.contact_forces = np.zeros((len(self.feet_body), 3), dtype=float)
        # Reset deliberately removes the previous experiment's evidence.
        self.last_diagnostic: StepDiagnostic | None = None

    @staticmethod
    def _quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
        aw, ax, ay, az = a
        bw, bx, by, bz = b
        return np.array([
            aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
        ])

    @staticmethod
    def _quat_to_rotation(q: np.ndarray) -> np.ndarray:
        w, x, y, z = q
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
        ])

    def _advance_rotation(self, omega_mid: np.ndarray, dt: float) -> None:
        angle = float(np.linalg.norm(omega_mid)) * dt
        if angle < 1e-12:
            dq = np.r_[1.0, 0.5 * dt * omega_mid]
        else:
            axis = omega_mid / np.linalg.norm(omega_mid)
            dq = np.r_[math.cos(0.5*angle), axis * math.sin(0.5*angle)]
        # Body-frame angular velocity right-multiplies body-to-world attitude.
        self._quaternion = self._quat_multiply(self._quaternion, dq)
        self._quaternion /= np.linalg.norm(self._quaternion)
        self.rotation = self._quat_to_rotation(self._quaternion)

    def _contacts(self) -> tuple[np.ndarray, np.ndarray, tuple[ContactDiagnostic, ...]]:
        force_total = np.zeros(3)
        torque_body = np.zeros(3)
        forces = np.zeros((len(self.feet_body), 3), dtype=float)
        details = []
        c = self.config
        for index, r_body in enumerate(self.feet_body):
            r_world = self.rotation @ r_body
            p = self.position + r_world
            point_velocity = self.velocity + self.rotation @ np.cross(self.omega, r_body)
            normal = 0.0
            fxy = np.zeros(2)
            if p[2] < 0.0:
                normal = max(0.0, -c.contact_stiffness*p[2] - c.contact_damping*point_velocity[2])
                if normal > 0.0:
                    tangential = point_velocity[:2]
                    speed = float(np.linalg.norm(tangential))
                    # Smooth near zero but never exceed Coulomb's inequality.
                    fxy = -c.friction*normal*tangential / math.sqrt(
                        speed*speed + c.friction_velocity*c.friction_velocity
                    )
            force = np.array([fxy[0], fxy[1], normal])
            forces[index] = force
            force_total += force
            torque_body += np.cross(r_body, self.rotation.T @ force)
            details.append(ContactDiagnostic(
                index, _tuple(p), _tuple(point_velocity), max(0.0, -float(p[2])),
                float(normal), _tuple(fxy), _tuple(force)))
        self.contact_forces = forces
        return force_total, torque_body, tuple(details)

    def _single_step(self, targets: np.ndarray) -> None:
        c = self.config
        time_before = self.time
        position_before = self.position.copy()
        velocity_before = self.velocity.copy()
        omega_before = self.omega.copy()
        rotation_before = self.rotation.copy()
        speeds_before = self.rotor_speeds.copy()
        energy_before = self.energy()
        force, external_torque_body, contacts = self._contacts()

        requested = np.clip(targets, -c.max_speed, c.max_speed)
        torques = np.clip(c.servo_gain*(requested-self.rotor_speeds),
                          -c.max_torque, c.max_torque)
        # If numerical overshoot ever crosses the speed limit, torque only inward.
        torques = np.where((self.rotor_speeds >= c.max_speed) & (torques > 0), 0.0, torques)
        torques = np.where((self.rotor_speeds <= -c.max_speed) & (torques < 0), 0.0, torques)
        self.last_torques = torques.astype(float)

        h_body = self._angular_momentum_body()
        motor_reaction = -self.axes_body.T @ torques
        gyroscopic = -np.cross(self.omega, h_body)
        alpha = self._J_body_inv @ (
            external_torque_body + motor_reaction + gyroscopic
        )
        speed_dot = torques/c.rotor_inertia - self.axes_body @ alpha

        old_omega = self.omega.copy()
        old_speeds = self.rotor_speeds.copy()
        self.omega += c.dt * alpha
        self.rotor_speeds += c.dt * speed_dot
        self.rotor_angles += c.dt * 0.5*(old_speeds+self.rotor_speeds)
        self._advance_rotation(0.5*(old_omega+self.omega), c.dt)

        acceleration = force/c.mass + np.array([0.0, 0.0, -c.gravity])
        self.velocity += c.dt * acceleration
        self.position += c.dt * self.velocity
        self.time += c.dt
        motor_work = float(torques @ (0.5*(speeds_before+self.rotor_speeds))) * c.dt
        self.last_diagnostic = StepDiagnostic(
            float(time_before), float(c.dt), _tuple(targets), _tuple(requested),
            _tuple(position_before), _tuple(velocity_before), _tuple(omega_before),
            _tuple(rotation_before), _tuple(speeds_before), contacts, _tuple(force),
            _tuple(external_torque_body), _tuple(torques), _tuple(h_body),
            _tuple(motor_reaction), _tuple(gyroscopic), _tuple(alpha), _tuple(speed_dot),
            _tuple(acceleration), _tuple(self.omega), _tuple(self.rotor_speeds),
            _tuple(self.velocity), _tuple(self.position), float(energy_before),
            float(self.energy()), motor_work,
        )

    def step(self, targets: tuple[float, float] = (0.0, 0.0), steps: int = 1) -> None:
        """Advance ``steps`` fixed ticks toward signed relative rotor rates (rad/s)."""
        if isinstance(steps, bool) or not isinstance(steps, (int, np.integer)) or steps < 0:
            raise ValueError("steps must be a nonnegative integer")
        command = np.asarray(targets, dtype=float)
        if command.shape != (2,) or not np.all(np.isfinite(command)):
            raise ValueError("targets must contain two finite speeds")
        for _ in range(int(steps)):
            self._single_step(command)

    def _angular_momentum_body(self) -> np.ndarray:
        axial_rates = self.axes_body @ self.omega + self.rotor_speeds
        return self._J_body @ self.omega + self.axes_body.T @ (
            self.config.rotor_inertia * axial_rates
        )

    def diagnostic_residuals(self, diagnostic: StepDiagnostic | None = None) -> dict[str, np.ndarray | float]:
        """Recompute key equations from a captured tick; zeros mean agreement.

        Accepting an explicit record makes this a useful independent audit seam:
        callers/tests can alter a copied diagnostic and verify the check detects it.
        """
        d = self.last_diagnostic if diagnostic is None else diagnostic
        if d is None:
            raise ValueError("no completed physics tick")
        inertia = np.array([self.config.body_inertia_x, self.config.body_inertia_y,
                            self.config.body_inertia_z])
        alpha = (np.array(d.contact_torque_body_nm) + np.array(d.motor_reaction_nm)
                 + np.array(d.gyroscopic_nm)) / inertia
        velocity = np.array(d.velocity_before_mps) + d.dt_s*np.array(d.linear_acceleration_mps2)
        position = np.array(d.position_before_m) + d.dt_s*np.array(d.velocity_after_mps)
        rotor = np.array(d.rotor_speeds_before_rad_s) + d.dt_s*np.array(d.rotor_acceleration_rad_s2)
        return {
            "angular_acceleration": alpha-np.array(d.angular_acceleration_rad_s2),
            "velocity": velocity-np.array(d.velocity_after_mps),
            "position": position-np.array(d.position_after_m),
            "rotor_speed": rotor-np.array(d.rotor_speeds_after_rad_s),
            "energy_minus_motor_work": d.energy_after_j-d.energy_before_j-d.motor_work_j,
        }

    def angular_momentum_world(self) -> np.ndarray:
        """Return total chassis-plus-rotor angular momentum in world coordinates."""
        return self.rotation @ self._angular_momentum_body()

    def energy(self) -> float:
        """Return mechanical energy, including gravity and contact spring energy."""
        c = self.config
        axial_rates = self.axes_body @ self.omega + self.rotor_speeds
        value = (0.5*c.mass*float(self.velocity @ self.velocity)
                 + c.mass*c.gravity*self.position[2]
                 + 0.5*float(self.omega @ self._J_body @ self.omega)
                 + 0.5*c.rotor_inertia*float(axial_rates @ axial_rates))
        for r_body in self.feet_body:
            z = (self.position + self.rotation @ r_body)[2]
            if z < 0.0:
                value += 0.5*c.contact_stiffness*z*z
        return float(value)

    def snapshot(self) -> dict[str, float | int]:
        """Return JSON-safe scalar telemetry."""
        r = self.rotation
        # ZYX yaw-pitch-roll, robust at the pitch singularity.
        pitch = math.asin(float(np.clip(-r[2, 0], -1.0, 1.0)))
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
        rpm = self.rotor_speeds * (60.0/(2.0*math.pi))
        return {
            "t": float(self.time),
            "x": float(self.position[0]), "y": float(self.position[1]), "z": float(self.position[2]),
            "vx": float(self.velocity[0]), "vy": float(self.velocity[1]), "vz": float(self.velocity[2]),
            "roll_deg": math.degrees(roll), "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw),
            "wx": float(self.omega[0]), "wy": float(self.omega[1]), "wz": float(self.omega[2]),
            "left_rpm": float(rpm[0]), "right_rpm": float(rpm[1]),
            "energy": self.energy(), "contacts": int(np.count_nonzero(self.contact_forces[:, 2] > 0.0)),
        }
