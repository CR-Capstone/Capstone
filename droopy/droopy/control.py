"""Open-loop rotor targets, not a prescribed robot trajectory.

These experimental waveforms only command motor speed. There is no controller
that sets chassis position or guarantees a walking direction.
"""
from dataclasses import dataclass
import math


@dataclass
class Command:
    enabled: bool = False
    mode: str = "Alternating gait"
    spin: float = 220.0
    amplitude: float = 180.0
    frequency: float = 1.0
    bias: float = 0.0
    left: float = 220.0
    right: float = -220.0

    def raw_targets(self, time: float) -> tuple[float, float]:
        """Signed gait output before the physical rotor-speed limit."""
        if not self.enabled:
            return 0.0, 0.0
        if self.mode == "Manual rotors":
            return self.left, self.right
        wave = self.amplitude * math.sin(2 * math.pi * self.frequency * time)
        if self.mode == "Constant counterspin":
            wave = 0.0
        # One rotor's magnitude rises while the other's falls. Adding the same
        # signed perturbation to opposite base rates achieves this.
        return self.spin + wave + self.bias, -self.spin + wave + self.bias

    def targets(self, time: float, max_speed: float) -> tuple[float, float]:
        return tuple(max(-max_speed, min(max_speed, v)) for v in self.raw_targets(time))
