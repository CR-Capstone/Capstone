"""Bench sequences: timing, ramps and the return to zero."""
import pytest

from controls.sequences import TESTS, Sequence, Step, custom_ramp


def test_duration_is_the_sum_of_the_steps():
    sequence = Sequence("t", "", (Step(1.0), Step(2.5), Step(0.5)))
    assert sequence.duration() == pytest.approx(4.0)


def test_hold_steps_return_their_own_values():
    sequence = Sequence("t", "", (Step(2.0, 0.3, -0.4), Step(1.0, 0.1, 0.1)))
    assert sequence.sample(0.0) == (0.3, -0.4)
    assert sequence.sample(1.99) == (0.3, -0.4)
    assert sequence.sample(2.5) == (0.1, 0.1)


def test_ramp_interpolates_from_the_previous_step():
    sequence = Sequence("t", "", (Step(1.0, 0.2, 0.2), Step(2.0, 0.6, 0.0, ramp=True)))
    assert sequence.sample(2.0)[0] == pytest.approx(0.4)
    assert sequence.sample(2.0)[1] == pytest.approx(0.1)


def test_first_ramp_starts_from_zero():
    sequence = Sequence("t", "", (Step(2.0, 1.0, -1.0, ramp=True),))
    assert sequence.sample(1.0) == pytest.approx((0.5, -0.5))


def test_before_and_after_the_run_is_zero():
    sequence = Sequence("t", "", (Step(1.0, 0.5, 0.5),))
    assert sequence.sample(-1.0) == (0.0, 0.0)
    assert sequence.sample(1.0) == (0.0, 0.0)
    assert sequence.finished(1.0) is True
    assert sequence.finished(0.9) is False


def test_step_at_reports_the_running_step_then_none():
    sequence = Sequence("t", "", (Step(1.0, label="a"), Step(1.0, label="b")))
    assert sequence.step_at(0.5).label == "a"
    assert sequence.step_at(1.5).label == "b"
    assert sequence.step_at(2.5) is None


def test_custom_ramp_mirrors_the_second_motor():
    sequence = custom_ramp(0.4, 4.0, mirrored=True)
    assert sequence.sample(4.5) == pytest.approx((0.4, -0.4))
    assert sequence.sample(sequence.duration()) == (0.0, 0.0)


@pytest.mark.parametrize("sequence", TESTS, ids=[s.name for s in TESTS])
def test_shipped_tests_stay_in_range_and_end_at_zero(sequence):
    assert sequence.duration() > 0.0
    assert sequence.watch_for
    steps = max(2, round(sequence.duration() * 50))
    for index in range(steps + 1):
        demand = sequence.sample(sequence.duration() * index / steps)
        assert all(-1.0 <= value <= 1.0 for value in demand)
    assert sequence.sample(sequence.duration()) == (0.0, 0.0)


def test_every_shipped_test_has_a_unique_name():
    names = [sequence.name for sequence in TESTS]
    assert len(names) == len(set(names))
