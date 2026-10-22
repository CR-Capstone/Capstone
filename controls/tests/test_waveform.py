"""Gait maths, independent of Qt and of the serial link."""
import math

import pytest

from controls.waveform import SHAPES, Gait, shape


@pytest.mark.parametrize("name", SHAPES)
def test_shapes_stay_in_unit_range_and_repeat(name):
    for step in range(200):
        phase = step / 200.0 * 4.0 * math.pi
        value = shape(name, phase)
        assert -1.0 <= value <= 1.0
        assert shape(name, phase + 2.0 * math.pi) == pytest.approx(value, abs=1e-9)


def test_unknown_shape_rejected():
    with pytest.raises(ValueError):
        shape("noise", 0.0)


def test_counter_rotating_base_with_common_mode_wave():
    gait = Gait(spin=0.4, amplitude=0.1, frequency=1.0, ramp_s=0.0, phase_deg=90.0)
    m1, m2 = gait.raw(0.0)
    # Phase 90 degrees puts the sine at its peak, added to both motors.
    assert m1 == pytest.approx(0.5)
    assert m2 == pytest.approx(-0.3)


def test_offset_shifts_motor_two_only():
    gait = Gait(spin=0.0, amplitude=1.0, frequency=1.0, ramp_s=0.0,
                offset_deg=180.0, counter_rotate=False)
    m1, m2 = gait.raw(0.25)
    assert m1 == pytest.approx(1.0)
    assert m2 == pytest.approx(-1.0)


def test_ramp_scales_from_zero():
    gait = Gait(spin=1.0, amplitude=0.0, frequency=0.0, ramp_s=2.0)
    assert gait.sample(0.0)[0] == pytest.approx(0.0)
    assert gait.sample(1.0)[0] == pytest.approx(0.5)
    assert gait.sample(5.0)[0] == pytest.approx(1.0)


def test_sample_clamps_but_raw_does_not():
    gait = Gait(spin=0.9, amplitude=0.5, frequency=1.0, ramp_s=0.0, phase_deg=90.0)
    assert gait.raw(0.0)[0] == pytest.approx(1.4)
    assert gait.sample(0.0)[0] == 1.0


def test_disabled_gait_is_silent():
    gait = Gait(enabled=False, spin=1.0, ramp_s=0.0)
    assert gait.sample(3.0) == (0.0, 0.0)


def test_period():
    assert Gait(frequency=4.0).period() == pytest.approx(0.25)
    assert Gait(frequency=0.0).period() == 0.0


def test_sim_presets_never_reverse_a_rotor():
    """The droopy gait keeps its swing inside the counter-rotating base."""
    from controls.waveform import PRESETS

    for name, preset in PRESETS.items():
        steady = Gait(**{**preset.__dict__, "ramp_s": 0.0})
        period = steady.period() or 1.0
        for step in range(360):
            m1, m2 = steady.raw(period * step / 360.0)
            assert m1 > 0.0 > m2, name
            assert abs(m1) <= 1.0 and abs(m2) <= 1.0, name


def test_nine_pound_preset_matches_the_simulator():
    from controls.waveform import PRESETS

    gait = PRESETS["Droopy sim gait, 9 lb (0.83 Hz)"]
    max_speed = 374.67                       # rad/s, droopy README 9 lb table
    assert gait.spin * max_speed == pytest.approx(183.17, rel=0.01)
    assert gait.amplitude * max_speed == pytest.approx(149.87, rel=0.01)
    assert gait.frequency == pytest.approx(0.8326, rel=0.01)
