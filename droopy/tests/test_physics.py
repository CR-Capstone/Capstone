import json
import math
from dataclasses import replace

import numpy as np
import pytest

from droopy.physics import Config, Simulator


def free_space(**kwargs):
    sim = Simulator(Config(gravity=0.0, friction=0.0, **kwargs))
    sim.position[2] = 1.0
    return sim


def state(sim):
    return np.concatenate((
        sim.position, sim.velocity, sim.omega, sim.rotation.ravel(),
        sim.rotor_speeds, sim.rotor_angles, sim.last_torques,
    ))


def test_repeat_is_bitwise_deterministic():
    a, b = Simulator(), Simulator()
    for target, count in [((280.0, -240.0), 1500), ((0.0, 0.0), 700)]:
        a.step(target, count)
        b.step(target, count)
    np.testing.assert_array_equal(state(a), state(b))
    assert a.snapshot() == b.snapshot()


def test_resting_robot_settles_without_horizontal_propulsion():
    sim = Simulator()
    sim.step(steps=8000)
    assert abs(sim.position[0]) < 1e-14
    assert abs(sim.position[1]) < 1e-14
    assert abs(sim.velocity[0]) < 1e-14
    assert abs(sim.velocity[1]) < 1e-14
    assert abs(sim.position[2] - 0.0548147) < 2e-5
    assert np.linalg.norm(sim.velocity) < 1e-7


def test_motor_action_cannot_translate_without_external_forces():
    sim = free_space()
    initial_position = sim.position.copy()
    sim.step((320.0, -190.0), 3000)
    np.testing.assert_allclose(sim.velocity, 0.0, atol=0.0)
    np.testing.assert_allclose(sim.position, initial_position, atol=0.0)
    assert np.linalg.norm(sim.omega) > 1.0


def test_total_angular_momentum_conserved_in_free_space():
    sim = free_space()
    initial = sim.angular_momentum_world().copy()
    sim.step((350.0, -290.0), 4000)
    sim.step((-80.0, 210.0), 2500)
    np.testing.assert_allclose(sim.angular_momentum_world(), initial, atol=5e-12, rtol=0.0)


def test_each_contact_respects_coulomb_bound_and_is_unilateral():
    c = Config(friction=0.43)
    sim = Simulator(c)
    sim.position[2] -= 0.001
    sim.velocity[:] = [2.0, -1.0, -0.1]
    sim.step(steps=1)
    assert len(sim.contact_forces) == 4
    for force in sim.contact_forces:
        assert force[2] >= 0.0
        assert np.linalg.norm(force[:2]) <= c.friction * force[2] + 1e-12


def test_signed_counterrotors_and_axis_tilt_change_body_response():
    angled = free_space(tilt_deg=15.0)
    flat = free_space(tilt_deg=0.0)
    angled.step((300.0, -300.0), 1200)
    flat.step((300.0, -300.0), 1200)
    # Opposite signed rates cancel z momentum. Outward tilt leaves a y couple.
    assert abs(angled.omega[1]) > 2.0
    assert abs(angled.omega[1]) > abs(flat.omega[1]) + 2.0
    assert abs(angled.omega[2]) < 1e-8
    assert angled.rotor_speeds[0] > 0.0 > angled.rotor_speeds[1]


def test_equal_spin_reacts_mainly_about_vertical_axis():
    sim = free_space()
    sim.step((250.0, 250.0), 1200)
    assert sim.omega[2] < -5.0
    assert abs(sim.omega[0]) < 1e-8
    assert abs(sim.omega[1]) < 1e-8


def test_long_run_remains_finite_and_rotation_orthonormal():
    sim = Simulator()
    sim.step((340.0, -310.0), 12000)
    assert np.all(np.isfinite(state(sim)))
    np.testing.assert_allclose(sim.rotation.T @ sim.rotation, np.eye(3), atol=2e-12)
    assert np.linalg.det(sim.rotation) == pytest.approx(1.0, abs=2e-12)
    assert np.all(np.isfinite(sim.contact_forces))
    assert sim.position[2] > -0.1


@pytest.mark.parametrize("field,value", [
    ("dt", 0.0), ("mass", -1.0), ("tilt_deg", 90.0),
    ("friction", -0.1), ("rotor_inertia", 0.0),
    ("max_torque", math.inf), ("max_speed", math.nan),
    ("gravity", -0.01), ("contact_stiffness", 0.0),
])
def test_config_rejects_nonfinite_or_unphysical(field, value):
    config = Config()
    setattr(config, field, value)
    with pytest.raises(ValueError, match=field):
        config.validate()


def test_step_input_validation_and_torque_speed_limits():
    sim = free_space()
    with pytest.raises(ValueError):
        sim.step((1.0,))
    with pytest.raises(ValueError):
        sim.step((0.0, math.nan))
    with pytest.raises(ValueError):
        sim.step(steps=-1)
    sim.step((1e9, -1e9), 3000)
    assert np.max(np.abs(sim.last_torques)) <= sim.config.max_torque
    assert np.max(np.abs(sim.rotor_speeds)) <= sim.config.max_speed + 1e-8


def test_snapshot_contract_is_json_safe_and_energy_matches_method():
    sim = Simulator()
    sim.step((100.0, -100.0), 20)
    snap = sim.snapshot()
    required = {
        "t", "x", "y", "z", "vx", "vy", "vz", "roll_deg", "pitch_deg",
        "yaw_deg", "wx", "wy", "wz", "left_rpm", "right_rpm", "energy",
        "contacts",
    }
    assert required <= snap.keys()
    json.dumps(snap, allow_nan=False)
    assert snap["energy"] == sim.energy()
    assert isinstance(snap["contacts"], int)


def test_diagnostic_is_exact_immutable_tick_and_reset_clears_it():
    sim = Simulator()
    sim.step((120.0, -80.0))
    captured = sim.last_diagnostic
    assert captured.time_before_s == 0.0
    assert captured.targets_rad_s == (120.0, -80.0)
    assert max(np.linalg.norm(v) for k,v in sim.diagnostic_residuals(captured).items()
               if k != "energy_minus_motor_work") < 1e-12
    sim.step((0.0, 0.0), 10)
    assert captured.time_before_s == 0.0  # no mutable ndarray aliasing
    altered = replace(captured, velocity_after_mps=(1.0, 0.0, 0.0))
    assert np.linalg.norm(sim.diagnostic_residuals(altered)["velocity"]) > .9
    sim.reset()
    assert sim.last_diagnostic is None
    with pytest.raises(ValueError, match="no completed"):
        sim.diagnostic_residuals()


def test_contact_diagnostic_recomputes_force_law_from_pre_step_values():
    c = Config(friction=.43)
    sim = Simulator(c); sim.position[2] -= .001; sim.velocity[:]=[2,-1,-.1]
    sim.step()
    for row in sim.last_diagnostic.contacts:
        pz, vz = row.point_world[2], row.velocity_world[2]
        expected_n=max(0.,-c.contact_stiffness*pz-c.contact_damping*vz) if pz < 0 else 0.
        assert row.normal_n == pytest.approx(expected_n)
        assert math.hypot(*row.friction_n) <= c.friction*row.normal_n+1e-12


def test_energy_formula_accounts_for_internal_motor_work():
    sim = free_space()
    initial = sim.energy()
    motor_work = 0.0
    for _ in range(2000):
        old_speeds = sim.rotor_speeds.copy()
        sim.step((220.0, -180.0))
        motor_work += float(sim.last_torques @ (0.5*(old_speeds+sim.rotor_speeds))) * sim.config.dt
    # Midpoint work and semi-implicit dynamics agree to integration accuracy.
    assert sim.energy() - initial == pytest.approx(motor_work, rel=3e-3, abs=2e-4)
