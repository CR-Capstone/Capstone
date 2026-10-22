"""Frame construction must match what the ESC expects, bit for bit."""
import pytest

from controls import dshot


def test_known_frames_round_trip():
    # Throttle 1046, no telemetry: payload 0x82C, nibbles 8^2^C = 6 -> 0x82C6.
    word = dshot.frame(1046)
    assert word == 0x82C6
    assert dshot.decode(word) == (1046, False, True)


def test_telemetry_bit_changes_checksum():
    plain = dshot.frame(1046, telemetry=False)
    telem = dshot.frame(1046, telemetry=True)
    assert plain != telem
    assert dshot.decode(telem)[1] is True


def test_bidirectional_inverts_checksum():
    assert (dshot.frame(1046, bidirectional=True) & 0xF) == (~0x6 & 0xF)


def test_every_value_frames_and_decodes():
    for value in range(dshot.VALUE_MAX + 1):
        word = dshot.frame(value)
        assert 0 <= word <= 0xFFFF
        assert dshot.decode(word) == (value, False, True)


def test_value_out_of_range_rejected():
    with pytest.raises(ValueError):
        dshot.frame(2048)
    with pytest.raises(ValueError):
        dshot.frame(-1)


def test_unidirectional_map_spans_the_full_range():
    mapping = dshot.MotorMap(deadband=0.0)
    assert mapping.value(0.0) == dshot.DISARMED
    assert mapping.value(1.0) == dshot.VALUE_MAX
    assert mapping.value(-1.0) == dshot.VALUE_MAX  # sign is not representable
    assert mapping.value(0.5) == dshot.MIN_THROTTLE + round(0.5 * dshot.FULL_SPAN)


def test_three_d_map_splits_the_range():
    mapping = dshot.MotorMap(three_d=True, deadband=0.0)
    assert mapping.value(1.0) == dshot.FORWARD_MAX
    assert mapping.value(-1.0) == dshot.REVERSE_MAX
    assert mapping.value(0.0) == dshot.NEUTRAL_3D
    assert mapping.value(0.5) > dshot.FORWARD_MIN
    assert mapping.value(-0.5) < dshot.NEUTRAL_3D


def test_three_d_map_matches_ramp_test_py():
    """remote-dshot's known-good script: 1049 slowest forward, 48 slowest reverse."""
    mapping = dshot.MotorMap(three_d=True, deadband=0.0)
    assert mapping.value(0.5) == 1548
    assert mapping.value(-0.5) == 548
    assert mapping.value(1e-4) == 1049
    assert mapping.value(-1e-4) == 48
    assert mapping.idle == 1048


def test_zero_demand_is_idle_not_disarm():
    assert dshot.MotorMap(three_d=True).value(0.0) == dshot.NEUTRAL_3D
    assert dshot.MotorMap(three_d=False).value(0.0) == dshot.DISARMED
    assert dshot.MotorMap(three_d=True).value(float("nan")) == dshot.NEUTRAL_3D


def test_deadband_limit_and_invert():
    mapping = dshot.MotorMap(three_d=True, deadband=0.1, limit=0.5, invert=True)
    assert mapping.value(0.05) == dshot.NEUTRAL_3D
    assert mapping.value(1.0) < dshot.NEUTRAL_3D  # inverted into reverse
    assert mapping.value(-1.0) == dshot.FORWARD_MIN + round(0.5 * dshot.FORWARD_SPAN)


def test_nan_and_overrange_are_safe():
    mapping = dshot.MotorMap(three_d=True, deadband=0.0)
    assert mapping.value(float("nan")) == dshot.NEUTRAL_3D
    assert mapping.value(9.0) == dshot.FORWARD_MAX
    assert mapping.value(-9.0) == dshot.REVERSE_MAX


def test_demand_of_inverts_value():
    mapping = dshot.MotorMap(three_d=True, deadband=0.0)
    for demand in (-1.0, -0.4, 0.0, 0.4, 1.0):
        assert mapping.demand_of(mapping.value(demand)) == pytest.approx(demand, abs=2e-3)
