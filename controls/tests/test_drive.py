"""Drive page: W walks, A / D steer by bias, both slew instead of stepping."""
import pytest
from PySide6.QtCore import Qt

from controls.pages.drive import DrivePage, slew


def test_slew_limits_the_step():
    assert slew(0.0, 1.0, 0.5, 0.1) == pytest.approx(0.05)
    assert slew(0.0, 0.01, 0.5, 0.1) == pytest.approx(0.01)
    assert slew(0.3, 0.0, 0.5, 0.1) == pytest.approx(0.25)


@pytest.fixture
def page(qt_app):
    from controls.engine import OutputEngine
    from controls.link import SerialLink

    link = SerialLink()
    page = DrivePage(OutputEngine(link), link)
    page.ramp.setValue(0.0)
    yield page
    page.engine.stop()


def test_spin_only_without_keys(page):
    page.sample(0.0)
    m1, m2 = page.sample(1.0)
    assert m1 == pytest.approx(0.49) and m2 == pytest.approx(-0.49)


def test_walk_ramps_in_at_the_slew_rate(page):
    page._held.add(Qt.Key.Key_W)
    page.sample(0.0)
    page.sample(0.2)
    assert page._amplitude == pytest.approx(0.1)   # 0.5 /s for 0.2 s
    for step in range(1, 20):
        page.sample(0.2 + step * 0.1)
    assert page._amplitude == pytest.approx(0.40)


def test_steer_adds_bias_to_both_rotors(page):
    page._held.add(Qt.Key.Key_D)
    page.sample(0.0)
    page.sample(1.0)
    assert page._bias == pytest.approx(0.04)
    m1, m2 = page.sample(1.0)
    assert m1 == pytest.approx(0.53) and m2 == pytest.approx(-0.45)
    page.invert_steer.setChecked(True)
    page.sample(2.0)
    assert page._bias == pytest.approx(-0.04)


def test_rearming_restarts_the_slew_from_zero(page):
    page._held.add(Qt.Key.Key_W)
    for step in range(30):
        page.sample(step * 0.1)
    page.sample(0.0)                               # clock restarted on arming
    assert page._amplitude == 0.0
