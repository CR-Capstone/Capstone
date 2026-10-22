"""Common shape for every control page.

A page is a widget plus, optionally, a waveform. When the page is shown the
app calls :meth:`ControlPage.sample` through the engine at the transmit rate.
Pages never write to the serial port themselves.
"""
from __future__ import annotations

from PySide6.QtWidgets import QWidget

from ..engine import OutputEngine
from ..link import SerialLink


class ControlPage(QWidget):
    """Base page. Subclasses set the class attributes and override `sample`."""

    title = "Page"
    stage = 0
    summary = ""
    provides_output = True

    def __init__(self, engine: OutputEngine, link: SerialLink) -> None:
        super().__init__()
        self.engine = engine
        self.link = link

    def sample(self, t: float) -> tuple[float, float]:
        """Signed demand in [-1, 1] for motor 1 and motor 2 at time ``t``."""
        return 0.0, 0.0

    def on_frame(self, frame) -> None:
        """Called for every transmitted frame while this page is active."""

    def on_activated(self) -> None:
        """Page became the output source. The engine is already disarmed."""

    def on_deactivated(self) -> None:
        """Page lost the output. Reset anything latched, such as held keys."""


def reverse_warning(engine: OutputEngine, negative: tuple[bool, bool]) -> str:
    """Flag a motor that is commanded negative but cannot send reverse.

    Outside the 3-D range a DShot value carries magnitude only, so a negative
    demand leaves the app as forward throttle. That is correct when the ESC
    itself is wired or configured reversed, and a trap otherwise.
    """
    bad = [
        index + 1
        for index in (0, 1)
        if negative[index] and not engine.config.maps[index].three_d
    ]
    if not bad:
        return ""
    names = " and ".join(f"M{n}" for n in bad)
    return (
        f"{names} commanded negative with no 3-D range: it leaves as forward "
        f"throttle. Set 3-D on the link page, or reverse the ESC."
    )


def zero_crossing_warning(crosses: tuple[bool, bool]) -> str:
    """Flag a motor whose demand changes sign within a gait cycle.

    Each crossing makes the ESC brake a loaded rotor through 3-D neutral and
    spin it up the other way: a current spike and the likeliest desync. The
    droopy gait never needs it; keep the wave and bias inside the spin base.
    """
    bad = [index + 1 for index in (0, 1) if crosses[index]]
    if not bad:
        return ""
    names = " and ".join(f"M{n}" for n in bad)
    return (
        f"{names} reverses direction every cycle. Keep amplitude + |bias| "
        f"below the spin base."
    )
