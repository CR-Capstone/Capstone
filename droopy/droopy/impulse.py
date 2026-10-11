"""Per-tick force and motion integration for the straight-travel display.

This module deliberately consumes immutable physics diagnostics rather than live
simulator state.  All accumulated quantities are SI-valued Python scalars or
immutable tuples, so snapshots can safely cross a UI/JSON boundary (after the
usual dataclass-to-dict conversion).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Any


Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class ImpulseSnapshot:
    """Immutable summary of force integration since the accumulator reset.

    Positive/negative directional totals are signed: positive totals are >= 0,
    negative totals are <= 0, and their sum is the corresponding net total.
    Straightness and path quantities are measured in the horizontal world plane.
    ``samples`` rows use the columns named by ``sample_columns``.
    """

    time_s: float
    ticks: int
    mass_kg: float
    gravity_mps2: float
    # Unit XY direction obtained by horizontally projecting initial body -X.
    reference_xy: tuple[float, float]
    contact_impulse_ns: Vector3
    gravity_impulse_ns: Vector3
    net_impulse_ns: Vector3
    momentum_change_kg_mps: Vector3
    residual_ns: Vector3
    displacement_m: Vector3
    velocity_mps: Vector3
    along_positive_ns: float
    along_negative_ns: float
    lateral_positive_ns: float
    lateral_negative_ns: float
    along_displacement_m: float
    lateral_displacement_m: float
    max_abs_cross_track_m: float
    path_length_m: float
    straightness_ratio: float | None
    # Samples are (time, along impulse, lateral impulse, along position,
    # lateral position), with units (s, N*s, N*s, m, m).
    samples: tuple[tuple[float, float, float, float, float], ...]
    initial_position_m: Vector3
    initial_velocity_mps: Vector3
    predicted_position_m: Vector3
    predicted_velocity_mps: Vector3
    position_prediction_error_m: Vector3
    velocity_prediction_error_mps: Vector3
    along_velocity_mps: float
    lateral_velocity_mps: float
    horizontal_net_displacement_m: float


def _vector3(value: Any, name: str) -> Vector3:
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain three finite numbers") from exc
    if len(result) != 3 or not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain three finite numbers")
    return result  # type: ignore[return-value]


def _add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _subtract(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a: Vector3, scalar: float) -> Vector3:
    return (a[0] * scalar, a[1] * scalar, a[2] * scalar)


def _dot(a: Vector3, b: Vector3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


class ImpulseAccumulator:
    """Integrate every completed physics tick without assuming a zero state."""

    def __init__(self, sample_interval_s: float = 0.02, max_samples: int = 600):
        if (not math.isfinite(float(sample_interval_s))
                or float(sample_interval_s) <= 0.0):
            raise ValueError("sample_interval_s must be positive and finite")
        if isinstance(max_samples, bool) or not isinstance(max_samples, int) or max_samples <= 0:
            raise ValueError("max_samples must be a positive integer")
        self._sample_interval_s = float(sample_interval_s)
        self._max_samples = max_samples
        self.reset()

    def reset(self) -> None:
        """Discard all prior experiment state while retaining sample settings."""
        self._ticks = 0
        self._duration = 0.0
        self._mass: float | None = None
        self._gravity: float | None = None
        self._reference: Vector3 | None = None
        self._lateral: Vector3 | None = None
        self._initial_position: Vector3 | None = None
        self._latest_position: Vector3 | None = None
        self._initial_velocity: Vector3 | None = None
        self._latest_velocity: Vector3 | None = None
        self._predicted_position: Vector3 | None = None
        self._predicted_velocity: Vector3 | None = None
        self._contact: Vector3 = (0.0, 0.0, 0.0)
        self._gravity_impulse: Vector3 = (0.0, 0.0, 0.0)
        self._net: Vector3 = (0.0, 0.0, 0.0)
        self._along_positive = 0.0
        self._along_negative = 0.0
        self._lateral_positive = 0.0
        self._lateral_negative = 0.0
        self._max_cross_track = 0.0
        self._path_length = 0.0
        self._samples: deque[tuple[float, ...]] = deque(maxlen=self._max_samples)
        self._last_sample_time: float | None = None

    def record(self, diagnostic: Any, mass: float, gravity: float) -> None:
        """Accumulate one completed ``StepDiagnostic``.

        ``mass`` and ``gravity`` are explicit because they are configuration,
        not diagnostic fields.  They must remain constant until ``reset``.
        """
        mass = float(mass)
        gravity = float(gravity)
        if not math.isfinite(mass) or mass <= 0.0:
            raise ValueError("mass must be positive and finite")
        if not math.isfinite(gravity) or gravity < 0.0:
            raise ValueError("gravity must be nonnegative and finite")
        if self._mass is not None and (mass != self._mass or gravity != self._gravity):
            raise ValueError("mass and gravity cannot change without reset")

        dt = float(diagnostic.dt_s)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("diagnostic.dt_s must be positive and finite")
        before_position = _vector3(diagnostic.position_before_m, "position_before_m")
        after_position = _vector3(diagnostic.position_after_m, "position_after_m")
        before_velocity = _vector3(diagnostic.velocity_before_mps, "velocity_before_mps")
        after_velocity = _vector3(diagnostic.velocity_after_mps, "velocity_after_mps")
        contact_force = _vector3(diagnostic.contact_force_n, "contact_force_n")

        if self._ticks == 0:
            rotation = tuple(float(item) for item in diagnostic.rotation_before)
            if len(rotation) != 9 or not all(math.isfinite(item) for item in rotation):
                raise ValueError("rotation_before must contain nine finite numbers")
            # Flattened row-major matrix: body X is its first column.  Travel's
            # fixed reference is initial body -X, projected into world XY.
            rx, ry = -rotation[0], -rotation[3]
            norm = math.hypot(rx, ry)
            if norm <= 1e-15:
                raise ValueError("initial body -X has no horizontal direction")
            self._reference = (rx / norm, ry / norm, 0.0)
            self._lateral = (-self._reference[1], self._reference[0], 0.0)
            self._mass, self._gravity = mass, gravity
            self._initial_position = before_position
            self._initial_velocity = before_velocity
            self._predicted_position = before_position
            self._predicted_velocity = before_velocity

        contact_impulse = _scale(contact_force, dt)
        gravity_impulse = (0.0, 0.0, -mass * gravity * dt)
        net_impulse = _add(contact_impulse, gravity_impulse)
        self._contact = _add(self._contact, contact_impulse)
        self._gravity_impulse = _add(self._gravity_impulse, gravity_impulse)
        self._net = _add(self._net, net_impulse)

        assert self._reference is not None and self._lateral is not None
        along = _dot(net_impulse, self._reference)
        lateral = _dot(net_impulse, self._lateral)
        if along >= 0.0:
            self._along_positive += along
        else:
            self._along_negative += along
        if lateral >= 0.0:
            self._lateral_positive += lateral
        else:
            self._lateral_negative += lateral

        # Match Simulator's semi-implicit update exactly: update v, then x.
        assert self._predicted_velocity is not None and self._predicted_position is not None
        self._predicted_velocity = _add(self._predicted_velocity, _scale(net_impulse, 1.0 / mass))
        self._predicted_position = _add(
            self._predicted_position, _scale(self._predicted_velocity, dt)
        )

        if self._latest_position is None:
            path_step = _subtract(after_position, before_position)
        else:
            path_step = _subtract(after_position, self._latest_position)
        self._path_length += math.hypot(path_step[0], path_step[1])
        self._latest_position = after_position
        self._latest_velocity = after_velocity
        self._ticks += 1
        self._duration += dt

        displacement = _subtract(after_position, self._initial_position)  # type: ignore[arg-type]
        along_displacement = _dot(displacement, self._reference)
        lateral_displacement = _dot(displacement, self._lateral)
        self._max_cross_track = max(self._max_cross_track, abs(lateral_displacement))
        if (self._last_sample_time is None
                or self._duration - self._last_sample_time >= self._sample_interval_s - 1e-15):
            self._samples.append((
                self._duration,
                self._along_positive + self._along_negative,
                self._lateral_positive + self._lateral_negative,
                along_displacement,
                lateral_displacement,
            ))
            self._last_sample_time = self._duration

    def snapshot(self) -> ImpulseSnapshot | None:
        """Return a detached immutable view, or ``None`` before the first tick."""
        if self._ticks == 0:
            return None

        assert self._mass is not None and self._reference is not None and self._lateral is not None
        assert self._initial_position is not None and self._latest_position is not None
        assert self._initial_velocity is not None and self._latest_velocity is not None
        assert self._predicted_position is not None and self._predicted_velocity is not None
        displacement = _subtract(self._latest_position, self._initial_position)
        momentum = _scale(_subtract(self._latest_velocity, self._initial_velocity), self._mass)
        residual = _subtract(momentum, self._net)
        net_distance = math.hypot(displacement[0], displacement[1])
        ratio = None if self._path_length == 0.0 else net_distance / self._path_length
        return ImpulseSnapshot(
            time_s=self._duration,
            ticks=self._ticks,
            mass_kg=self._mass,
            gravity_mps2=self._gravity,  # type: ignore[arg-type]
            reference_xy=(self._reference[0], self._reference[1]),
            contact_impulse_ns=self._contact,
            gravity_impulse_ns=self._gravity_impulse,
            net_impulse_ns=self._net,
            momentum_change_kg_mps=momentum,
            residual_ns=residual,
            displacement_m=displacement,
            velocity_mps=self._latest_velocity,
            along_positive_ns=self._along_positive,
            along_negative_ns=self._along_negative,
            lateral_positive_ns=self._lateral_positive,
            lateral_negative_ns=self._lateral_negative,
            along_displacement_m=_dot(displacement, self._reference),
            lateral_displacement_m=_dot(displacement, self._lateral),
            max_abs_cross_track_m=self._max_cross_track,
            path_length_m=self._path_length,
            straightness_ratio=ratio,
            samples=tuple(self._samples),
            initial_position_m=self._initial_position,
            initial_velocity_mps=self._initial_velocity,
            predicted_position_m=self._predicted_position,
            predicted_velocity_mps=self._predicted_velocity,
            position_prediction_error_m=_subtract(
                self._predicted_position, self._latest_position
            ),
            velocity_prediction_error_mps=_subtract(
                self._predicted_velocity, self._latest_velocity
            ),
            along_velocity_mps=_dot(self._latest_velocity, self._reference),
            lateral_velocity_mps=_dot(self._latest_velocity, self._lateral),
            horizontal_net_displacement_m=net_distance,
        )
