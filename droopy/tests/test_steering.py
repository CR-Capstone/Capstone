"""Evidence for the bounded, default-model-only steering experiment."""
import json
import math
from dataclasses import asdict

import numpy as np
import pytest

from droopy.control import Command
from droopy.physics import Simulator
from droopy.steering import DirectionController, MAX_BIAS


def physical_state(sim):
    return np.concatenate((
        sim.position, sim.velocity, sim.omega, sim.rotation.ravel(),
        sim.rotor_speeds, sim.rotor_angles, sim.last_torques,
    ))


def angle_error(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def run(desired, assisted, steps=6000):
    sim = Simulator()
    base = Command(enabled=True)
    controller = DirectionController()
    for _ in range(steps):
        command = controller.update(sim, base, desired, 1.0) if assisted else base
        sim.step(command.targets(sim.time, sim.config.max_speed))
    travel = math.degrees(math.atan2(sim.position[1], sim.position[0]))
    return angle_error(desired, travel), travel, sim, controller


def test_update_only_returns_command_and_state_is_json_safe():
    sim = Simulator()
    sim.step(steps=100)
    before = physical_state(sim).copy()
    before_time = sim.time
    base = Command(enabled=True, spin=201.0, amplitude=123.0, frequency=1.3, bias=4.0)
    controller = DirectionController()

    command = controller.update(sim, base, -150.0, 0.5)

    np.testing.assert_array_equal(physical_state(sim), before)
    assert sim.time == before_time
    assert base == Command(enabled=True, spin=201.0, amplitude=123.0,
                           frequency=1.3, bias=4.0)
    assert command is not base
    assert command.amplitude == pytest.approx(61.5)
    assert abs(command.bias - base.bias) <= MAX_BIAS * 0.5
    json.dumps(asdict(controller.state), allow_nan=False)
    assert "simulation yaw" in controller.state.uses_truth


def test_zero_strength_is_explicit_motor_braking_neutral():
    sim = Simulator()
    controller = DirectionController()
    base = Command(enabled=True, amplitude=180.0, bias=7.0)
    command = controller.update(sim, base, 90.0, 0.0)
    assert not command.enabled
    assert command.amplitude == 0.0
    assert command.targets(sim.time, sim.config.max_speed) == (0.0, 0.0)
    assert controller.state.strength == 0.0
    assert controller.state.command_bias == 7.0


def test_bias_is_bounded_rate_limited_and_resettable():
    sim = Simulator()
    base = Command(enabled=True)
    controller = DirectionController()
    first = controller.update(sim, base, 0.0, 1.0)
    assert abs(first.bias) <= 80.0 * sim.config.dt + 1e-12
    for _ in range(3000):
        sim.step(first.targets(sim.time, sim.config.max_speed))
        first = controller.update(sim, base, 0.0, 1.0)
        assert abs(first.bias) <= MAX_BIAS
    controller.reset()
    assert controller.state.command_bias == 0.0
    again = controller.update(sim, base, 0.0, 1.0)
    assert abs(again.bias) <= 80.0 * sim.config.dt + 1e-12


@pytest.mark.parametrize("desired", [-160.0, -140.0])
def test_default_model_direction_evidence_improves_over_open_loop(desired):
    """Two demands on the empirically controllable side improve, not converge.

    This intentionally compares displacement direction with the same open-loop
    gait.  The modest threshold documents limited steering authority rather
    than claiming path tracking.
    """
    open_error, _, _, _ = run(desired, assisted=False)
    assisted_error, _, sim, controller = run(desired, assisted=True)
    assert np.linalg.norm(sim.position[:2]) > 0.05
    assert assisted_error < open_error - 4.0
    assert controller.state.travel_angle_deg is not None
    assert controller.state.speed_mps >= 0.0


def test_repeat_is_deterministic_and_invalid_inputs_fail():
    a = run(-150.0, assisted=True, steps=2000)
    b = run(-150.0, assisted=True, steps=2000)
    np.testing.assert_array_equal(physical_state(a[2]), physical_state(b[2]))
    assert a[3].state == b[3].state
    controller = DirectionController()
    sim = Simulator()
    with pytest.raises(ValueError, match="finite"):
        controller.update(sim, Command(), math.nan, 1.0)
    with pytest.raises(ValueError, match="finite"):
        controller.update(sim, Command(), 0.0, math.inf)
