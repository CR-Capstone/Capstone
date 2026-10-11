import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest
from PySide6.QtWidgets import QApplication

from droopy.control import Command
from droopy.lesson import GuidedLesson
from droopy.physics import Config, Simulator


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


def test_signals_disabled_preview_and_quarter_cycle(app):
    lesson = GuidedLesson()
    sim = Simulator()
    disabled = Command(enabled=False, frequency=2.0)
    lesson.update_state(sim, None, disabled)
    assert "PREVIEW ONLY" in lesson.provenance_label.text()
    assert "(0, 0) rad/s" in lesson.pending_label.text()
    assert not lesson.waveform.isVisible()
    assert not lesson.equation_label.isVisible()

    experiments, steps, inspections = [], [], []
    lesson.experiment_requested.connect(experiments.append)
    lesson.step_requested.connect(steps.append)
    lesson.inspect_requested.connect(inspections.append)
    lesson.findChild(type(lesson.capture_button), "lesson_no_grip").click()
    lesson.step_button.click()
    lesson.select_stage(4); lesson.inspect_button.click()
    assert experiments == ["no_grip"]
    assert steps == [pytest.approx(0.125)]
    assert inspections == ["contacts"]
    lesson.close()


def test_capture_freezes_equation_and_pending_updates_separately(app):
    lesson = GuidedLesson(); sim = Simulator()
    consumed = Command(enabled=True, spin=220, amplitude=180, frequency=1)
    sim.step(consumed.raw_targets(0.0))
    lesson.update_state(sim, consumed, consumed)
    # A diagnostic is not silently adopted: reading stays under user control.
    assert "PREVIEW ONLY" in lesson.provenance_label.text()
    assert lesson.capture_current()
    lesson.select_stage(2)
    frozen_equation = lesson.equation_label.text()
    frozen_stamp = lesson.provenance_label.text()
    assert "FROZEN previous step" in frozen_stamp
    assert "0.000000" in frozen_stamp

    edited = Command(enabled=True, spin=410, amplitude=10, frequency=3)
    sim.step(edited.raw_targets(sim.time))
    lesson.update_state(sim, edited, edited)
    assert lesson.equation_label.text() == frozen_equation
    assert lesson.provenance_label.text() == frozen_stamp
    assert "410.000" in lesson.pending_label.text()
    assert lesson.equation_label.textFormat().name == "PlainText"
    assert lesson.equation_label.styleSheet().find("monospace") >= 0

    lesson.live_check.setChecked(True)
    lesson.update_state(sim, edited, edited)
    assert lesson.provenance_label.text() != frozen_stamp
    lesson.close()


def test_targets_projection_and_impulses_are_exact(app):
    lesson = GuidedLesson(); sim = Simulator(Config(gravity=0.0))
    command = Command(enabled=True, spin=200, amplitude=50, frequency=1, bias=5)
    sim.time = .25
    expected = command.raw_targets(sim.time)
    assert expected == pytest.approx((255.0, -145.0))
    lesson.update_state(sim, None, command)
    assert "+255.000" in lesson.pending_label.text()
    assert "-145.000" in lesson.pending_label.text()

    # Construct real diagnostics and independently check the accumulation seam.
    sim.position[2] = sim.config.foot_drop - .001
    sim.velocity[:] = [0.2, -0.1, -0.05]
    sim.step(expected)
    d = sim.last_diagnostic
    expected_world = np.asarray(d.contact_force_n) * d.dt_s
    rotation = np.asarray(d.rotation_before).reshape(3, 3)
    expected_body = np.dot(np.asarray(d.contact_force_n), rotation[:, 0]) * d.dt_s
    lesson.record_tick(d)
    assert lesson.accumulated_world_impulse_ns == pytest.approx(expected_world)
    assert lesson.accumulated_body_forward_impulse_ns == pytest.approx(expected_body)
    lesson.update_state(sim, command, command)
    lesson.capture_current(); lesson.select_stage(5)
    forward = float(np.dot(sim.velocity, sim.rotation[:, 0]))
    assert f"{forward:+.4f}" in lesson.equation_label.text()
    assert "not mΔv" in lesson.detail.text()

    lesson.reset_history()
    assert lesson.accumulated_world_impulse_ns == (0.0, 0.0, 0.0)
    assert lesson.accumulated_body_forward_impulse_ns == 0.0
    assert lesson.recorded_ticks == 0
    lesson.close()


def test_zero_friction_has_zero_horizontal_contact_impulse(app):
    lesson = GuidedLesson()
    sim = Simulator(Config(friction=0.0, gravity=0.0))
    sim.position[2] -= .002
    sim.velocity[:] = [1.0, -0.5, -0.1]
    sim.step((0.0, 0.0))
    assert sim.last_diagnostic.contact_force_n[:2] == pytest.approx((0.0, 0.0))
    lesson.record_tick(sim.last_diagnostic)
    assert lesson.accumulated_world_impulse_ns[:2] == pytest.approx((0.0, 0.0))
    lesson.close()


def test_rotor_actions_come_from_actual_torque_and_speed(app):
    lesson = GuidedLesson(); sim = Simulator(Config(gravity=0.0))
    sim.position[2] = 1.0
    command = Command(enabled=True, mode="Manual rotors", left=100, right=-100)
    sim.rotor_speeds[:] = [20.0, -20.0]
    sim.step(command.raw_targets(sim.time))
    lesson.update_state(sim, command, command)
    description = lesson.rotor_bars.accessibleDescription()
    assert "left" in description and "speeding up" in description
    assert "right" in description and "speeding up" in description
    lesson.close()
