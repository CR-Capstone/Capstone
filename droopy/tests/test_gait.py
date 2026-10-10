"""Regression for the illustrative demo, not calibration against real Droopy."""
import numpy as np

from droopy.control import Command
from droopy.physics import Config, Simulator


def test_default_demo_translates_and_changes_contact_without_tipping():
    sim = Simulator()
    command = Command(enabled=True)
    counts = set()
    tilt = 0.0
    for i in range(8000):
        sim.step(command.targets(sim.time, sim.config.max_speed))
        if i % 20 == 0:
            snap = sim.snapshot()
            if sim.time > 1:
                counts.add(snap["contacts"])
            tilt = max(tilt, abs(snap["roll_deg"]), abs(snap["pitch_deg"]))
    assert np.linalg.norm(sim.position[:2]) > .05
    assert min(counts) < 4
    assert tilt < 15


def test_frictionless_floor_cannot_propel_horizontal_motion():
    sim = Simulator(Config(friction=0.0))
    command = Command(enabled=True)
    for _ in range(3000):
        sim.step(command.targets(sim.time,sim.config.max_speed))
    np.testing.assert_array_equal(sim.position[:2], [0.,0.])
    np.testing.assert_array_equal(sim.velocity[:2], [0.,0.])


def test_nonzero_angular_momentum_error_converges_with_timestep():
    errors = []
    for dt in (.0005, .00025):
        sim = Simulator(Config(dt=dt, gravity=0))
        sim.position[2] = 2
        sim.omega[:] = [.5, .7, .9]
        sim.rotor_speeds[:] = [150, -100]
        initial = sim.angular_momentum_world().copy()
        sim.step((220, -180), round(1 / dt))
        errors.append(np.linalg.norm(sim.angular_momentum_world() - initial))
    assert errors[0] < 2e-5
    assert errors[1] < .6 * errors[0]


def test_contact_force_rows_remain_aligned_to_feet():
    sim = Simulator()
    assert sim.contact_forces.shape == sim.feet_body.shape
    sim.step()
    assert sim.contact_forces.shape == sim.feet_body.shape
    assert sim.snapshot()["contacts"] == 0
    sim.step(steps=100)
    assert sim.snapshot()["contacts"] == 4
