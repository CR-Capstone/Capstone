"""Open-loop gait waveforms. Pure maths, no Qt, no serial.

This mirrors ``droopy.control.Command``: a counter-rotating base pair with a
common-mode periodic perturbation added to both rotors. One rotor's magnitude
rises while the other's falls, which is what shifts a droopy-style robot's
contact pattern and makes it translate.

Everything here is normalised: demands are signed fractions of full rotor
authority in [-1, 1]. The mapping to DShot values lives in
:class:`controls.dshot.MotorMap`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

TWO_PI = 2.0 * math.pi
SHAPES = ("sine", "triangle", "square", "sawtooth")


def shape(name: str, phase: float) -> float:
    """Unit-amplitude periodic function of ``phase`` in radians."""
    turns = (phase / TWO_PI) % 1.0
    if name == "sine":
        return math.sin(TWO_PI * turns)
    if name == "triangle":
        return 1.0 - 4.0 * abs(((turns + 0.25) % 1.0) - 0.5)
    if name == "square":
        return 1.0 if turns < 0.5 else -1.0
    if name == "sawtooth":
        return 2.0 * ((turns + 0.5) % 1.0) - 1.0
    raise ValueError(f"unknown waveform {name!r}")


@dataclass
class Gait:
    """Continuous two-motor waveform, sampled by the output engine each tick.

    ``spin`` is the steady counter-rotating base. ``amplitude`` is the periodic
    term added to both motors; ``phase_deg`` rotates where in the spin that
    term peaks, which is the translation heading. ``offset_deg`` phase-shifts
    motor 2 only, for differential experiments — leave it at 0 to match the
    simulator's gait.
    """

    enabled: bool = True
    shape_name: str = "sine"
    spin: float = 0.35
    amplitude: float = 0.20
    frequency: float = 1.0
    bias: float = 0.0
    phase_deg: float = 0.0
    offset_deg: float = 0.0
    counter_rotate: bool = True
    ramp_s: float = 1.5

    def envelope(self, t: float) -> float:
        """Spin-up scaling so arming never steps straight to full authority."""
        if self.ramp_s <= 0.0:
            return 1.0
        return max(0.0, min(1.0, float(t) / self.ramp_s))

    def raw(self, t: float) -> tuple[float, float]:
        """Signed demand before clamping, so the UI can show saturation."""
        if not self.enabled:
            return 0.0, 0.0
        env = self.envelope(t)
        angle = TWO_PI * self.frequency * float(t) + math.radians(self.phase_deg)
        wave1 = self.amplitude * shape(self.shape_name, angle)
        wave2 = self.amplitude * shape(
            self.shape_name, angle + math.radians(self.offset_deg)
        )
        base2 = -self.spin if self.counter_rotate else self.spin
        return (
            env * (self.spin + wave1 + self.bias),
            env * (base2 + wave2 + self.bias),
        )

    def sample(self, t: float) -> tuple[float, float]:
        """Clamped demand, which is what the engine actually transmits."""
        return tuple(max(-1.0, min(1.0, v)) for v in self.raw(t))

    def period(self) -> float:
        """Seconds per cycle, or 0 when stationary."""
        return 1.0 / self.frequency if self.frequency > 0.0 else 0.0
