import math

from droopy.control import Command


def test_disabled_commands_braking():
    assert Command().targets(2,450) == (0.,0.)


def test_alternating_changes_opposite_magnitudes():
    c = Command(enabled=True,spin=200,amplitude=50,frequency=1)
    assert c.targets(.25,450) == (250.,-150.)
    assert c.targets(.75,450) == (150.,-250.)


def test_constant_and_manual_clamped():
    assert Command(enabled=True,mode="Constant counterspin").targets(.25,450) == (220.,-220.)
    assert Command(enabled=True,mode="Manual rotors",left=600,right=-600).targets(0,450) == (450,-450)
