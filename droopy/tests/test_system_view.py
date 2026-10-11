import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from droopy.explain import KEYS
from droopy.physics import Config, Simulator
from droopy.system_view import SystemView


def _tick(config=None):
    sim = Simulator(config or Config())
    # Produce nonzero contact, friction, motor, and integration values in one
    # immutable diagnostic so the diagram has meaningful substitutions.
    sim.position[2] -= 0.001
    sim.velocity[:] = (0.2, -0.1, -0.03)
    sim.omega[:] = (0.4, -0.3, 0.2)
    sim.step((1000.0, -80.0))
    return sim.last_diagnostic, sim.config


def test_tick_displays_consumed_numerics_equations_frames_and_bounds():
    app = QApplication.instance() or QApplication([])
    diagnostic, config = _tick()
    view = SystemView()
    view.update_tick(diagnostic, config)

    assert "0.000000 → 0.000500 s" in view.state_label.text()
    control = view.blocks["control"].body.text()
    assert "clamp" in control and "+450" in control and "-80" in control

    motor = view.blocks["motor"].body.text()
    assert "error = requested − s₀" in motor
    assert "+0.8" in motor and "-0.8" in motor
    assert "NO direct thrust" in motor

    rotation = view.blocks["rotation"].body.text()
    assert "H = Jb·ω" in rotation
    assert "−Σ aᵢuᵢ" in rotation
    assert "−ω×H" in rotation
    assert "Euler coupling term" in rotation
    for value in diagnostic.angular_acceleration_rad_s2:
        assert f"{value:+.5g}" in rotation

    contacts = view.blocks["contacts"].body.text()
    assert "pre-step" in contacts and "|Ft| ≤ μN" in contacts
    assert contacts.count("foot ") == 4
    assert f"{diagnostic.contacts[0].normal_n:.5g}" in contacts

    translation = view.blocks["translation"].body.text()
    assert "Fworld = Ffloor + (0,0,−mg)" in translation
    assert "v₁ = v₀ + aΔt" in translation and "x₁ = x₀ + v₁Δt" in translation
    for value in diagnostic.position_after_m:
        assert f"{value:+.5g}" in translation

    feedback = view.blocks["energy"].body.text()
    assert "FEEDBACK BRANCH ↺ NEXT TICK" in feedback
    assert "pre-step state R₀, not the new attitude" in feedback
    assert "body→world" in feedback
    view.close()


def test_freeze_capture_none_and_reset_keep_complete_tick_config_pairs():
    app = QApplication.instance() or QApplication([])
    view = SystemView()
    first, config = _tick(Config(max_torque=0.4, servo_gain=0.01))
    view.update_tick(first, config)
    view.freeze.setChecked(True)
    first_text = view.blocks["motor"].body.text()

    # Mutating the caller's live Config after freezing cannot rewrite history.
    config.max_torque = 9.0
    second, second_config = _tick(Config(dt=0.001, max_torque=0.7))
    view.update_tick(second, second_config)
    assert view.diagnostic == first
    assert view.config.max_torque == 0.4
    assert view.blocks["motor"].body.text() == first_text

    # Capture explicitly replaces the held pair with the newest pair.
    view.capture_latest()
    assert view.diagnostic == second
    assert view.config.max_torque == 0.7
    assert "Δt 0.001 s" in view.state_label.text()

    # None means no latest data. Freeze still protects the captured evidence;
    # Capture latest then accurately presents the empty snapshot.
    view.update_tick(None, second_config)
    assert view.diagnostic == second
    view.capture_latest()
    assert view.diagnostic is None
    assert "no completed tick" in view.state_label.text()

    view.reset()
    assert not view.freeze.isChecked()
    assert view.latest_diagnostic is None and view.diagnostic is None
    assert "Live · no completed tick" == view.state_label.text()
    view.close()


def test_render_and_click_blocks_emit_explain_keys(tmp_path):
    app = QApplication.instance() or QApplication([])
    diagnostic, config = _tick()
    view = SystemView()
    view.update_tick(diagnostic, config)
    view.resize(380, 1600)
    view.show()
    app.processEvents()

    emitted = []
    view.inspect_requested.connect(emitted.append)
    for key, _label in KEYS:
        QTest.mouseClick(view.blocks[key], Qt.MouseButton.LeftButton)
    assert emitted == [key for key, _label in KEYS]
    assert view.grab().save(str(tmp_path / "system-view.png"))
    assert not view.grab().isNull()
    view.close()
