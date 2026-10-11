"""Experimental world-direction assist for the illustrative simulator.

This is a heading controller, not a trajectory controller.  The default model
usually travels approximately along body ``-X``.  Simulation truth is therefore
used to turn the requested world direction into an error from body ``-X`` and a
small, rate-limited differential gait bias.  A positive bias was empirically
observed to turn that nominal axis clockwise for the default geometry.

The constants below are deliberately fixed and calibrated only for the default
illustrative :class:`~droopy.physics.Config`.  Contact transitions can dominate
the weak steering authority, so this does not guarantee a path or even
convergence for every direction.  In particular, large turns and changed
physical parameters should be treated as experiments, not robot controls.
Only a new :class:`~droopy.control.Command` is returned; simulator state is
never edited.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math

from .control import Command

# Empirical default-model calibration (bias is in rad/s).
BIAS_PER_ERROR_DEG = 0.40
MAX_BIAS = 45.0
BIAS_RATE_PER_S = 80.0
TRAVEL_SPEED_THRESHOLD_MPS = 0.005
USES_TRUTH = "simulation yaw and planar velocity; nominal travel is body -X"


def _wrap_deg(angle: float) -> float:
    """Wrap degrees to [-180, 180), including very large finite inputs."""
    return (angle + 180.0) % 360.0 - 180.0


@dataclass
class SteeringState:
    """JSON-safe diagnostics for the most recent controller update."""

    requested_angle_deg: float = 0.0
    strength: float = 0.0
    heading_error_deg: float = 0.0
    command_bias: float = 0.0
    travel_angle_deg: float | None = None
    speed_mps: float = 0.0
    uses_truth: str = USES_TRUTH


class DirectionController:
    """Bounded best-effort world heading assist for the default simulation."""

    def __init__(self) -> None:
        self.state = SteeringState()
        self._correction = 0.0
        self._last_time: float | None = None

    def reset(self) -> None:
        """Forget the prior rate-limit state and clear diagnostics."""
        self._correction = 0.0
        self._last_time = None
        self.state = SteeringState()

    def update(
        self, sim, base: Command, angle_deg: float, strength: float
    ) -> Command:
        """Return a command aimed toward ``angle_deg`` in world XY coordinates.

        ``strength`` is clamped to [0, 1] and scales alternating-gait amplitude.
        Zero strength returns a disabled command, asking the speed servos to
        brake toward zero.  Yaw and velocity are read as truth feedback; no
        simulator array, force, pose, or velocity is modified.
        """
        angle_deg = float(angle_deg)
        strength = float(strength)
        if not math.isfinite(angle_deg) or not math.isfinite(strength):
            raise ValueError("angle_deg and strength must be finite")
        strength = max(0.0, min(1.0, strength))

        snap = sim.snapshot()
        yaw_deg = float(snap["yaw_deg"])
        speed = math.hypot(float(snap["vx"]), float(snap["vy"]))
        travel_angle = None
        if speed >= TRAVEL_SPEED_THRESHOLD_MPS:
            travel_angle = _wrap_deg(math.degrees(math.atan2(snap["vy"], snap["vx"])))

        # The gait's nominal forward axis is body -X, i.e. yaw + 180 degrees.
        nominal_angle = _wrap_deg(yaw_deg + 180.0)
        error = _wrap_deg(angle_deg - nominal_angle)

        now = float(sim.time)
        if self._last_time is None or now < self._last_time:
            dt = float(sim.config.dt)
        else:
            # Duplicate calls at one tick do not continue slewing the output.
            dt = min(0.1, now - self._last_time)
        self._last_time = now

        if strength == 0.0:
            self._correction = 0.0
            output = replace(base, enabled=False, amplitude=0.0, bias=base.bias)
        else:
            target = max(-MAX_BIAS, min(MAX_BIAS, -BIAS_PER_ERROR_DEG * error))
            target *= strength
            delta_limit = BIAS_RATE_PER_S * dt
            delta = max(-delta_limit, min(delta_limit, target - self._correction))
            self._correction += delta
            amplitude = base.amplitude * strength
            output = replace(
                base,
                enabled=base.enabled,
                amplitude=amplitude,
                bias=base.bias + self._correction,
            )

        self.state = SteeringState(
            requested_angle_deg=angle_deg,
            strength=strength,
            heading_error_deg=error,
            command_bias=float(output.bias),
            travel_angle_deg=travel_angle,
            speed_mps=speed,
        )
        return output
