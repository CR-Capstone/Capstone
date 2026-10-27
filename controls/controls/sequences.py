"""Canned bench tests. Pure maths, no Qt, no serial.

A sequence is a list of timed steps, each holding or ramping to a pair of
signed demands. The bench page runs one at a time against the engine clock and
returns to zero when it ends.

These are bring-up checks, in the order they are usually needed: does the ESC
arm, which way does each motor turn, where does it start moving, does the
counter-rotating pair actually counter-rotate, how does it respond to a step.
Run them with the robot restrained.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Step:
    """Hold, or ramp from the previous step's end to these demands."""

    seconds: float
    m1: float = 0.0
    m2: float = 0.0
    ramp: bool = False
    label: str = ""

    def describe(self) -> str:
        if self.label:
            return self.label
        verb = "ramp to" if self.ramp else "hold"
        return f"{verb} M1 {self.m1:+.0%} / M2 {self.m2:+.0%}"


@dataclass(frozen=True)
class Sequence:
    """A named test, watched for one specific thing."""

    name: str
    watch_for: str
    steps: tuple[Step, ...]

    def duration(self) -> float:
        return sum(step.seconds for step in self.steps)

    def sample(self, t: float) -> tuple[float, float]:
        """Demand at ``t`` seconds into the run; zero before and after."""
        t = float(t)
        if t < 0.0:
            return 0.0, 0.0
        previous = (0.0, 0.0)
        elapsed = 0.0
        for step in self.steps:
            if t < elapsed + step.seconds:
                if not step.ramp:
                    return step.m1, step.m2
                fraction = (t - elapsed) / step.seconds if step.seconds > 0 else 1.0
                return (
                    previous[0] + (step.m1 - previous[0]) * fraction,
                    previous[1] + (step.m2 - previous[1]) * fraction,
                )
            elapsed += step.seconds
            previous = (step.m1, step.m2)
        return 0.0, 0.0

    def step_at(self, t: float) -> Step | None:
        """The step running at ``t``, or None once the sequence has ended."""
        elapsed = 0.0
        for step in self.steps:
            if t < elapsed + step.seconds:
                return step
            elapsed += step.seconds
        return None

    def finished(self, t: float) -> bool:
        return t >= self.duration()


def custom_ramp(target: float, seconds: float, mirrored: bool) -> Sequence:
    """The parametric ramp from the bench page's own fields."""
    second = -target if mirrored else target
    return Sequence(
        name="Custom ramp",
        watch_for="Whatever you set the fields below to.",
        steps=(
            Step(seconds, target, second, ramp=True, label=f"ramp to {target:+.0%}"),
            Step(1.0, target, second, label="hold"),
            Step(min(2.0, seconds), 0.0, 0.0, ramp=True, label="back to zero"),
        ),
    )


# remote-dshot's ramp_test.py steps one count every 50 ms: 500 counts is 25 s.
RAMP_TEST_LEG_S = 25.0

TESTS: tuple[Sequence, ...] = (
    Sequence(
        "Arm check",
        "Arming sends 0 for 1.6 s, then neutral for 0.4 s. Both ESCs play "
        "their armed tone and nothing turns.",
        (Step(3.0, 0.0, 0.0, label="neutral"),),
    ),
    Sequence(
        "Ramp test (ramp_test.py)",
        "The known-good remote-dshot ramp: both motors forward up to 50% "
        "(1548) and back, 2 s at neutral, then reverse up to 50% (548) and "
        "back. Same values and timing as the script.",
        (
            Step(RAMP_TEST_LEG_S, 0.5, 0.5, ramp=True, label="forward ramp up"),
            Step(RAMP_TEST_LEG_S, 0.0, 0.0, ramp=True, label="forward ramp down"),
            Step(2.0, 0.0, 0.0, label="neutral"),
            Step(RAMP_TEST_LEG_S, -0.5, -0.5, ramp=True, label="reverse ramp up"),
            Step(RAMP_TEST_LEG_S, 0.0, 0.0, ramp=True, label="reverse ramp down"),
        ),
    ),
    Sequence(
        "Direction — motor 1",
        "Which way motor 1 turns at a low forward command.",
        (
            Step(1.0, 0.0, 0.0, label="settle"),
            Step(3.0, 0.10, 0.0, label="M1 at 10%"),
            Step(1.0, 0.0, 0.0, label="stop"),
        ),
    ),
    Sequence(
        "Direction — motor 2",
        "Which way motor 2 turns at a low forward command.",
        (
            Step(1.0, 0.0, 0.0, label="settle"),
            Step(3.0, 0.0, 0.10, label="M2 at 10%"),
            Step(1.0, 0.0, 0.0, label="stop"),
        ),
    ),
    Sequence(
        "Counter-rotate check",
        "The pair turning opposite ways. If they match, the 3-D mapping or the "
        "ESC direction is wrong.",
        (
            Step(1.0, 0.0, 0.0, label="settle"),
            Step(4.0, 0.12, -0.12, label="M1 +12% / M2 -12%"),
            Step(1.0, 0.0, 0.0, label="stop"),
        ),
    ),
    Sequence(
        "Deadband sweep",
        "The command where each motor first moves. Watch the readout at the "
        "moment it breaks away.",
        (
            Step(20.0, 0.20, 0.20, ramp=True, label="0 to 20% over 20 s"),
            Step(2.0, 0.0, 0.0, ramp=True, label="back to zero"),
        ),
    ),
    Sequence(
        "Step response",
        "How fast each motor reaches speed, and whether the ESC desyncs on a "
        "hard step.",
        (
            Step(1.5, 0.0, 0.0, label="zero"),
            Step(1.5, 0.20, 0.20, label="step to 20%"),
            Step(1.5, 0.0, 0.0, label="back to zero"),
            Step(1.5, 0.35, 0.35, label="step to 35%"),
            Step(1.5, 0.0, 0.0, label="back to zero"),
        ),
    ),
    Sequence(
        "Slow ramp to 50%",
        "Current draw and vibration climbing through the range, with time to "
        "abort.",
        (
            Step(8.0, 0.50, 0.50, ramp=True, label="0 to 50% over 8 s"),
            Step(2.0, 0.50, 0.50, label="hold 50%"),
            Step(3.0, 0.0, 0.0, ramp=True, label="back to zero"),
        ),
    ),
    Sequence(
        "Reverse check",
        "Reverse on both motors. Needs the 3-D range; without it this runs "
        "forward instead.",
        (
            Step(1.0, 0.0, 0.0, label="settle"),
            Step(3.0, -0.12, -0.12, label="both at -12%"),
            Step(1.0, 0.0, 0.0, label="stop"),
        ),
    ),
)
