import math
import numpy as np
import pytest
from droopy.presets import robot_preset, POUNDS_TO_KG
from droopy.physics import Config, Simulator


def test_nine_pounds_scales_mass_geometry_inertia_and_commands():
    base=Config();config,command=robot_preset()
    ratio=9*POUNDS_TO_KG/base.mass;scale=ratio**(1/3)
    assert config.mass == pytest.approx(9*POUNDS_TO_KG)
    assert config.rotor_offset_y == pytest.approx(base.rotor_offset_y*scale)
    assert config.rotor_inertia == pytest.approx(base.rotor_inertia*ratio*scale**2)
    assert config.max_torque == pytest.approx(base.max_torque*ratio*scale)
    assert command.frequency == pytest.approx(1/math.sqrt(scale))
    assert config.contact_stiffness == pytest.approx(base.contact_stiffness*ratio/scale)
    assert robot_preset('3lb')[0] == base
    config.mass=99
    assert robot_preset()[0].mass !=99


def test_nine_pound_demo_stable_with_contact_driven_motion():
    config,command=robot_preset();command.enabled=True
    sim=Simulator(config);max_tilt=0
    for i in range(6000):
        sim.step(command.raw_targets(sim.time))
        if i%20==0:
            d=sim.snapshot();max_tilt=max(max_tilt,abs(d['roll_deg']),abs(d['pitch_deg']))
    assert np.isfinite(sim.position).all()
    assert np.linalg.norm(sim.position[:2])>.04
    assert max_tilt<15


def test_nine_pounds_no_grip_cannot_translate_horizontally():
    config,command=robot_preset();config.friction=0;command.enabled=True
    sim=Simulator(config)
    for _ in range(1000):sim.step(command.raw_targets(sim.time))
    np.testing.assert_array_equal(sim.position[:2],[0,0])
